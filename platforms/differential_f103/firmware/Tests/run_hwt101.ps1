$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
Push-Location $workspace
try {
    & gcc -std=c99 -Wall -Wextra -Werror -IHardware Tests/test_hwt101.c Hardware/HWT101_Parser.c -lm -o Tests/test_hwt101.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
    & ./Tests/test_hwt101.exe
    if ($LASTEXITCODE -ne 0) { throw 'HWT101 parser test failed' }
} finally { Pop-Location }
