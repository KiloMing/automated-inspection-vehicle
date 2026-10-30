"""Parse SDFormat CLI XML with its known Gazebo extension namespace."""
import re
import xml.etree.ElementTree as ET


def parse_sdf(text):
    # gz sdf prints custom prefixed attributes but may omit their xmlns declaration.
    # Bind the prefix externally; never insert the expressed_in attribute itself.
    text = re.sub(r'^\s*<\?xml[^?]*\?>', '', text)
    wrapper = ET.fromstring(
        '<validation xmlns:gz="http://gazebosim.org/schema">' + text + '</validation>')
    if len(wrapper) != 1 or wrapper[0].tag != 'sdf':
        raise ValueError('Expected one SDF document')
    return wrapper[0]
