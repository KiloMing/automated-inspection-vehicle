// Copyright <2020> [Copyright rossihwang@gmail.com]
#include <camsense_x1/camsense_x1.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <chrono>
#include <stdexcept>
#include <thread>

// Camsense X1 stream decoding and scan assembly; no ROS dependency.

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace camsense {
inline double wrap(double a) { a=std::fmod(a,360.0); return a<0 ? a+360.0 : a; }
struct Point { double angle; uint16_t distance; uint8_t quality; };
struct Packet { std::array<Point,8> points; double step; };
class Decoder {
 public:
  void append(const uint8_t* data, size_t n) { buffer_.insert(buffer_.end(),data,data+n); }
  size_t pending() const { return buffer_.size(); }
  void clear() { buffer_.clear(); }
  bool next(Packet& p) {
    while(buffer_.size()>=2) {
      if(buffer_[0]!=3 || buffer_[1]!=8) {buffer_.erase(buffer_.begin());continue;}
      if(buffer_.size()<36) return false;
      if(buffer_[34]!=0x55 || buffer_[35]!=0xaa) {buffer_.erase(buffer_.begin());continue;}
      auto word=[this](size_t i){return uint16_t(buffer_[i] | (uint16_t(buffer_[i+1])<<8));};
      const double start=word(4)/64.0-640.0, end=word(30)/64.0-640.0;
      if(start<0 || start>=360 || end<0 || end>=360) {buffer_.erase(buffer_.begin());continue;}
      p.step=wrap(end-start)/8.0;
      // Bytes 32..33 are intentionally NOT validated: no verified checksum formula.
      for(size_t i=0;i<8;++i) p.points[i]={wrap(start+p.step*i),word(6+i*3),buffer_[8+i*3]};
      buffer_.erase(buffer_.begin(),buffer_.begin()+36);
      return true;
    }
    return false;
  }
 private:
  std::vector<uint8_t> buffer_;
};
struct Scan {
  std::vector<float> ranges, intensities;
  double stamp=0, period=0, time_increment=0;
};
class ScanBuilder {
 public:
  ScanBuilder(size_t count,bool clockwise,double offset,double min_range,double max_range,int quality)
  : count_(count), clockwise_(clockwise), offset_(offset), min_(min_range), max_(max_range), quality_(quality) {clear_ranges();}
  size_t index(double raw) const {
    return std::min(count_-1,static_cast<size_t>(coordinate(raw)*static_cast<double>(count_)/360.0));
  }
  void reset() {seen_=false; started_=false; clear_ranges();}
  double period() const {return period_;}
  bool add(const Point& p,double time,Scan& out) {
    const double u=coordinate(p.angle);
    const double phase=clockwise_ ? wrap(360-u) : u;
    bool ready=false;
    bool boundary_point=false;
    if(seen_ && time<last_time_) reset();
    if(seen_ && last_phase_-phase>180) {
      boundary_point=clockwise_ && phase==0.0;
      if(started_ && boundary_point) store(p);
      const double gap=360-last_phase_+phase;
      const double boundary=last_time_+(time-last_time_)*(360-last_phase_)/gap;
      const double elapsed=boundary-start_time_;
      if(started_ && elapsed>0.02 && elapsed<2.0) {
        period_=elapsed;
        out.ranges=ranges_;out.intensities=intensities_;out.period=elapsed;
        // Ascending ROS angle order reverses acquisition order for clockwise scans.
        out.time_increment=(clockwise_?-1.0:1.0)*elapsed/count_;
        out.stamp=clockwise_?boundary:start_time_;
        ready=true;
      }
      clear_ranges();start_time_=boundary;started_=true;
    }
    seen_=true;last_phase_=phase;last_time_=time;
    if(started_ && !boundary_point) store(p);
    return ready;
  }
 private:
  void store(const Point& p) {
    const double range=p.distance/1000.0;
    if(p.distance!=0x8000 && p.distance>0 && p.quality>=quality_ && p.quality>0 && range>=min_ && range<=max_) {
      const auto i=index(p.angle);
      if(range<ranges_[i]) {ranges_[i]=static_cast<float>(range);intensities_[i]=p.quality;}
    }
  }
  double coordinate(double raw) const {return wrap((clockwise_?-raw:raw)-offset_+180.0);}
  void clear_ranges() {ranges_.assign(count_,std::numeric_limits<float>::infinity());intensities_.assign(count_,0);}
  size_t count_;bool clockwise_;double offset_,min_,max_;int quality_;
  bool seen_=false,started_=false;
  double last_phase_=0,last_time_=0,start_time_=0,period_=0.19;
  std::vector<float> ranges_,intensities_;
};
}  // namespace camsense

CamsenseX1::CamsenseX1(const std::string &name, const rclcpp::NodeOptions &options)
  : Node(name, options), frame_id_("laser_link"), port_("/dev/ttyUSB0"),
    baud_(115200), angle_offset_(0), state_(State::SYNC1), canceled_(false),
    speed_(0), start_angle_(0.0), end_angle_(0.0),
    rate_(std::chrono::milliseconds(32)) {
  create_parameter();  // Read launch overrides BEFORE opening the serial port.
  ranges_.resize(400);
  intensities_.resize(400);
  reset_data();
  scan_pub_=create_publisher<sensor_msgs::msg::LaserScan>("/scan",10);
  serial_ptr_=std::make_shared<serial::Serial>(
    port_,baud_,serial::Timeout::simpleTimeout(100));
  if(!serial_ptr_->isOpen()) throw std::runtime_error("Cannot open "+port_);
  RCLCPP_INFO(get_logger(),"Opened %s, baud=%d, frame=%s",port_.c_str(),baud_,frame_id_.c_str());
  RCLCPP_WARN(get_logger(),"Checksum formula unknown: only framing is checked. Beam times are estimates, not hardware timestamps.");
  thread_=std::thread([this]() { parse(); });
}

CamsenseX1::~CamsenseX1() {
  canceled_.store(true);
  if(thread_.joinable()) thread_.join();
  if(serial_ptr_ && serial_ptr_->isOpen()) serial_ptr_->close();
}

void CamsenseX1::parse() {
  const bool clockwise=get_parameter("clockwise").as_bool();
  // Preserve the old angle_offset sign: old code reversed (raw-offset).
  const double offset=clockwise ? -double(angle_offset_) : double(angle_offset_);
  camsense::Decoder decoder;
  camsense::ScanBuilder builder(400,clockwise,offset,0.08,8.0,1);
  std::array<uint8_t,4096> bytes;
  auto last_data=std::chrono::steady_clock::now();
  while(rclcpp::ok() && !canceled_.load()) {
    try {
      const size_t requested=std::min(bytes.size(),std::max(size_t(1),serial_ptr_->available()));
      const size_t n=serial_ptr_->read(bytes.data(),requested);
      const auto current=std::chrono::steady_clock::now();
      if(current-last_data>std::chrono::milliseconds(500)) {
        decoder.clear(); builder.reset();
      }
      if(n==0) continue;
      last_data=current;
      decoder.append(bytes.data(),n);
      const double received=now().seconds();
      camsense::Packet packet;
      while(decoder.next(packet)) {
        // 8N1: 10 bits/byte. Approximate queued packet receive times.
        // USB/OS latency remains; this does not perform motion compensation.
        const double packet_time=received-decoder.pending()*10.0/baud_;
        const double point_dt=packet.step/360.0*builder.period();
        for(size_t i=0;i<8;++i) {
          camsense::Scan scan;
          if(!builder.add(packet.points[i],packet_time-(7-i)*point_dt,scan)) continue;
          constexpr double pi=3.14159265358979323846;
          sensor_msgs::msg::LaserScan msg;
          msg.header.frame_id=frame_id_;
          msg.header.stamp=rclcpp::Time(static_cast<int64_t>(scan.stamp*1e9),get_clock()->get_clock_type());
          msg.angle_min=-pi;
          msg.angle_increment=2.0*pi/400.0;
          msg.angle_max=msg.angle_min+399*msg.angle_increment;
          msg.scan_time=static_cast<float>(scan.period);
          // Clockwise scan sorted in ascending angle has reversed acquisition time.
          // Do NOT change the sign alone; verify downstream RF2O compatibility.
          msg.time_increment=static_cast<float>(scan.time_increment);
          msg.range_min=0.08f; msg.range_max=8.0f;
          msg.ranges=std::move(scan.ranges);
          msg.intensities=std::move(scan.intensities);
          scan_pub_->publish(msg);
        }
      }
    } catch(const std::exception &e) {
      RCLCPP_ERROR(get_logger(),"Serial/scan processing failed: %s. Check device and restart node.",e.what());
      return;
    }
  }
}

void CamsenseX1::reset_data() {
  std::fill(ranges_.begin(),ranges_.end(),std::numeric_limits<float>::infinity());
  std::fill(intensities_.begin(),intensities_.end(),0.0f);
}

void CamsenseX1::create_parameter() {
  // Startup-only parameters avoid serial reconfiguration and worker-thread races.
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only=true;
  frame_id_=declare_parameter<std::string>("frame_id","laser_link",descriptor);
  port_=declare_parameter<std::string>("port","/dev/ttyUSB0",descriptor);
  baud_=declare_parameter<int>("baud",115200,descriptor);
  angle_offset_=declare_parameter<int>("angle_offset",0,descriptor);
  declare_parameter<bool>("clockwise",true,descriptor);
  if(frame_id_.empty() || port_.empty() || baud_<=0)
    throw std::invalid_argument("frame_id/port must be nonempty and baud must be positive");
}

bool CamsenseX1::handle_parameter(const rclcpp::Parameter &param) {
  // Kept for compatibility with the original header; no dynamic callback installed.
  (void)param;
  return false;
}
