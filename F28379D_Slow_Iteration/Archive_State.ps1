$ErrorActionPreference = 'Stop'
$packageDirectory = [IO.Path]::GetFullPath($PSScriptRoot)
$stateDirectory = [IO.Path]::GetFullPath((Join-Path $packageDirectory 'State'))
if (-not $stateDirectory.StartsWith($packageDirectory + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'State directory is outside the hardware package.'
}
$names = @('Controller_Input_History.mat', 'Active_PWM_Sequence.mat', 'Active_PWM_Sequence.csv', 'Iteration_History.csv')
$files = @($names | ForEach-Object { Join-Path $stateDirectory $_ } | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf })
if ($files.Count -eq 0) { exit 0 }
$archiveName = (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + [guid]::NewGuid().ToString('N').Substring(0,8)
$archiveDirectory = [IO.Path]::GetFullPath((Join-Path (Join-Path $stateDirectory 'Previous') $archiveName))
if (-not $archiveDirectory.StartsWith($stateDirectory + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Archive directory is outside State.'
}
New-Item -ItemType Directory -Path $archiveDirectory | Out-Null
foreach ($file in $files) {
    Move-Item -LiteralPath $file -Destination (Join-Path $archiveDirectory ([IO.Path]::GetFileName($file))) -ErrorAction Stop
}
Write-Output "Previous controller state preserved in $archiveDirectory"
