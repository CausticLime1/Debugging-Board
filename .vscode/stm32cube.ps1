[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("configure", "build", "flash", "flash-run")]
    [string]$Action,

    [string]$Preset = "Debug",

    [string]$ConnectPort = "SWD",

    [int]$FrequencyKHz = 4000,

    [string]$ElfPath,

    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

$workspaceRoot = Split-Path -Parent $PSScriptRoot
$bundleStorePath = Join-Path $workspaceRoot ".settings/bundles.store.json"

if (-not (Test-Path $bundleStorePath)) {
    throw "Cannot find bundle metadata at '$bundleStorePath'."
}

$bundleStore = Get-Content -Raw $bundleStorePath | ConvertFrom-Json
$bundleRoot = Join-Path $env:LOCALAPPDATA "stm32cube\bundles"

function Get-BundleVersion {
    param([string]$Name)

    $bundle = $bundleStore.bundles | Where-Object { $_.name -eq $Name } | Select-Object -First 1
    if (-not $bundle) {
        throw "Bundle '$Name' is not listed in .settings/bundles.store.json."
    }

    return $bundle.version
}

function Get-BundleExecutable {
    param(
        [string]$Name,
        [string]$RelativePath
    )

    $version = Get-BundleVersion -Name $Name
    $path = Join-Path $bundleRoot (Join-Path $Name (Join-Path $version $RelativePath))

    if (-not (Test-Path $path)) {
        throw "Expected bundle executable was not found: '$path'."
    }

    return $path
}

function Invoke-External {
    param(
        [string]$Executable,
        [string[]]$Arguments
    )

    $commandLine = @($Executable) + $Arguments
    Write-Host ("`n> " + ($commandLine -join " "))

    if ($DryRun) {
        return
    }

    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code $LASTEXITCODE."
    }
}

function Get-BuildElf {
    param([string]$BuildPreset)

    if ($ElfPath) {
        $resolvedElf = Resolve-Path $ElfPath -ErrorAction Stop
        return $resolvedElf.Path
    }

    $buildDir = Join-Path $workspaceRoot (Join-Path "build" $BuildPreset)
    if (-not (Test-Path $buildDir)) {
        throw "Build directory '$buildDir' does not exist. Run the build action first."
    }

    $elf = Get-ChildItem -Path $buildDir -Filter *.elf -File | Sort-Object LastWriteTimeUtc -Descending | Select-Object -First 1
    if (-not $elf) {
        throw "No ELF file was found in '$buildDir'."
    }

    return $elf.FullName
}

$cmakeExe = Get-BundleExecutable -Name "cmake" -RelativePath "bin/cmake.exe"
$gccBinDir = Split-Path -Parent (Get-BundleExecutable -Name "gnu-tools-for-stm32" -RelativePath "bin/arm-none-eabi-gcc.exe")
$ninjaBinDir = Split-Path -Parent (Get-BundleExecutable -Name "ninja" -RelativePath "bin/ninja.exe")
$programmerExe = Get-BundleExecutable -Name "programmer" -RelativePath "bin/STM32_Programmer_CLI.exe"

$env:Path = "$gccBinDir;$ninjaBinDir;$env:Path"

switch ($Action) {
    "configure" {
        Invoke-External -Executable $cmakeExe -Arguments @("--preset", $Preset)
    }

    "build" {
        $buildDir = Join-Path $workspaceRoot (Join-Path "build" $Preset)
        $cacheFile = Join-Path $buildDir "CMakeCache.txt"

        if (-not (Test-Path $cacheFile)) {
            Invoke-External -Executable $cmakeExe -Arguments @("--preset", $Preset)
        }

        Invoke-External -Executable $cmakeExe -Arguments @("--build", "--preset", $Preset)
    }

    "flash" {
        $resolvedElf = Get-BuildElf -BuildPreset $Preset
        Invoke-External -Executable $programmerExe -Arguments @(
            "-c", "port=$ConnectPort", "freq=$FrequencyKHz",
            "-w", $resolvedElf,
            "-v"
        )
    }

    "flash-run" {
        $resolvedElf = Get-BuildElf -BuildPreset $Preset
        Invoke-External -Executable $programmerExe -Arguments @(
            "-c", "port=$ConnectPort", "freq=$FrequencyKHz",
            "-w", $resolvedElf,
            "-v",
            "-rst"
        )
    }
}
