$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
Push-Location $workspace
try {
    & gcc -std=c99 -Wall -Wextra -Werror -IHardware Tests/test_motor_scan.c Hardware/MotorScan_Core.c -lm -o Tests/test_motor_scan.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor scan test compilation failed' }
    & ./Tests/test_motor_scan.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor scan tests failed' }
} finally { Pop-Location }
