# Build ArmForge and assemble a portable dist\ folder.
#
# Static linking is the default, so normally dist\ contains a single exe that
# runs on any Windows 10+ machine. When ARMFORGE_STATIC_RUNTIME=OFF the script
# still detects the MinGW runtime DLLs the exe imports and copies them next to
# the binary, so the folder keeps working either way.

param(
    [string]$Config = "Release",
    [switch]$SkipTests
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

function Find-Tool([string]$name) {
    $command = Get-Command $name -ErrorAction SilentlyContinue
    if (-not $command) { throw "$name not found on PATH" }
    return $command.Source
}

$gxx = Find-Tool "g++.exe"
$gxxDir = Split-Path -Parent $gxx
$objdump = Join-Path $gxxDir "objdump.exe"
$cmake = Find-Tool "cmake.exe"

Write-Host "[deploy] toolchain : $gxxDir"
Write-Host "[deploy] configure : $Config"

& $cmake -S . -B build -G Ninja "-DCMAKE_BUILD_TYPE=$Config" `
    -DARMFORGE_BUILD_TESTS=ON -DARMFORGE_BUILD_BENCH=ON
if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

& $cmake --build build
if ($LASTEXITCODE -ne 0) { throw "build failed" }

if (-not $SkipTests) {
    Write-Host "[deploy] unit tests"
    & "$root\build\bin\armforge_tests.exe"
    if ($LASTEXITCODE -ne 0) { throw "unit tests failed" }
}

if (Test-Path "$root\dist") { Remove-Item -Recurse -Force "$root\dist" }
New-Item -ItemType Directory -Path "$root\dist" | Out-Null

$artifacts = @("armforge.exe", "armforge_bench.exe")
foreach ($item in $artifacts) {
    Copy-Item "$root\build\bin\$item" "$root\dist\" -Force
}
Copy-Item "$root\README.md" "$root\dist\" -Force
if (Test-Path "$root\docs\arm.png") { Copy-Item "$root\docs\arm.png" "$root\dist\" -Force }

# Which non system DLLs does the exe import? Copy only those.
$systemDlls = @(
    "KERNEL32.dll", "msvcrt.dll", "USER32.dll", "GDI32.dll", "ADVAPI32.dll",
    "SHELL32.dll", "ole32.dll", "OLEAUT32.dll", "WS2_32.dll", "api-ms-win-crt-runtime-l1-1-0.dll"
)
$imports = & $objdump -p "$root\dist\armforge.exe" |
    Select-String "DLL Name:" |
    ForEach-Object { ($_ -split "DLL Name:")[1].Trim() } |
    Where-Object { $systemDlls -notcontains $_ }

if ($imports.Count -gt 0) {
    Write-Host "[deploy] runtime DLLs needed: $($imports -join ', ')"
    foreach ($dll in $imports) {
        $candidate = Join-Path $gxxDir $dll
        if (Test-Path $candidate) {
            Copy-Item $candidate "$root\dist\" -Force
        } else {
            Write-Warning "cannot find $dll next to g++ ($gxxDir); add it manually"
        }
    }
} else {
    Write-Host "[deploy] static runtime: no extra DLLs required"
}

Write-Host "[deploy] smoke test (runs the dist exe from a temp directory)"
$smoke = Join-Path $env:TEMP "armforge-smoke"
if (Test-Path $smoke) { Remove-Item -Recurse -Force $smoke }
New-Item -ItemType Directory -Path $smoke | Out-Null
Push-Location $smoke
try {
    & "$root\dist\armforge.exe" demo
    if ($LASTEXITCODE -ne 0) { throw "demo exited with $LASTEXITCODE" }
} finally {
    Pop-Location
}

Write-Host ""
Write-Host "[deploy] dist ready:"
Get-ChildItem "$root\dist" | Select-Object Name, Length | Format-Table -AutoSize | Out-String -Width 80 | Write-Host
Write-Host "[deploy] run it with: dist\armforge.exe demo"
