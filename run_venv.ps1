$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
if (!(Test-Path .venv/Scripts/python.exe)) { py -m venv .venv; if ($LASTEXITCODE) { exit $LASTEXITCODE } }
& ./.venv/Scripts/python.exe -m pip install -r requirements.txt
if ($LASTEXITCODE) { exit $LASTEXITCODE }
& ./.venv/Scripts/python.exe servo_controller.py
