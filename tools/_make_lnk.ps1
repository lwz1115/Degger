$ErrorActionPreference = "Stop"
$dir = "E:\CFKJ Work\G12Debugger"
$target = Join-Path $dir "launch_usv.bat"
$icon = Join-Path $dir "usv_icon.ico"
$desktop = [Environment]::GetFolderPath("Desktop")
Write-Output ("Desktop = " + $desktop)
$ws = New-Object -ComObject WScript.Shell
$lnkPath = Join-Path $desktop "_USV_DEBUG.lnk"
$lnk = $ws.CreateShortcut($lnkPath)
$lnk.TargetPath = $target
$lnk.WorkingDirectory = $dir
if (Test-Path $icon) { $lnk.IconLocation = $icon + ",0" }
$lnk.Description = "USV Debug Assistant"
$lnk.Save()
if (Test-Path $lnkPath) { Write-Output ("OK created: " + $lnkPath) } else { Write-Output "FAILED" }
