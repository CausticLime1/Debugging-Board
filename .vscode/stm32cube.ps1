[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("configure", "build", "flash", "flash-run", "reset", "codegen")]
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

    "reset" {
        Invoke-External -Executable $programmerExe -Arguments @(
            "-c", "port=$ConnectPort", "freq=$FrequencyKHz",
            "-rst"
        )
    }

    "codegen" {
        # Locate CubeMX using the same lookup as the ST VSCode extension:
        # 1. Read SoftwarePath from ~/.stm32cubemx/plugins/updater/updater.ini (written by the CubeMX installer)
        # 2. Fall back to the hardcoded default install path
        $cubemxExe = $null
        $updaterIni = Join-Path $env:USERPROFILE ".stm32cubemx\plugins\updater\updater.ini"
        if (Test-Path $updaterIni) {
            $softwarePath = Get-Content $updaterIni | Where-Object { $_ -match "^SoftwarePath=" } | Select-Object -First 1
            if ($softwarePath) {
                $candidate = Join-Path ($softwarePath -replace "^SoftwarePath=", "").Trim() "STM32CubeMX.exe"
                if (Test-Path $candidate) { $cubemxExe = $candidate }
            }
        }
        if (-not $cubemxExe) {
            $candidate = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeMX\STM32CubeMX.exe"
            if (Test-Path $candidate) { $cubemxExe = $candidate }
        }
        if (-not $cubemxExe) { throw "Could not find STM32CubeMX.exe. Install STM32CubeMX and ensure ~/.stm32cubemx/plugins/updater/updater.ini exists." }

        # Locate the .ioc file
        $iocFile = Get-ChildItem -Path $workspaceRoot -Filter "*.ioc" -File |
                   Select-Object -First 1 -ExpandProperty FullName
        if (-not $iocFile) { throw "No .ioc file found in '$workspaceRoot'." }

        Write-Host "CubeMX EXE : $cubemxExe"
        Write-Host "IOC file   : $iocFile"

        # Write a CubeMX batch script
        $batchScript = Join-Path $env:TEMP "cubemx_codegen.txt"
        @"
project generate
exit
"@ | Set-Content -Encoding ASCII $batchScript

        Write-Host ("`n> " + $cubemxExe + " `"$iocFile`" -q " + $batchScript)
        if (-not $DryRun) {
            $psi = New-Object System.Diagnostics.ProcessStartInfo
            $psi.FileName = $cubemxExe
            $psi.Arguments = "`"$iocFile`" -q `"$batchScript`""
            $psi.RedirectStandardOutput = $true
            $psi.RedirectStandardError = $true
            $psi.UseShellExecute = $false
            $psi.CreateNoWindow = $true

            $proc = New-Object System.Diagnostics.Process
            $proc.StartInfo = $psi
            $proc.Start() | Out-Null
            $proc.BeginErrorReadLine()  # drain stderr async to prevent deadlock

            $generated = 0
            while (-not $proc.StandardOutput.EndOfStream) {
                $line = $proc.StandardOutput.ReadLine()
                if ($line -match "Generated code: (.+)") {
                    $generated++
                    Write-Host "  [$generated] $($Matches[1])"
                } elseif ($line -match "^\s*(OK|KO)\s*$") {
                    Write-Host $line
                } elseif ($line -match "SWIPConfigModel") {
                    Write-Host "Known error suppressed (SWIPConfigModel)"
                } elseif ($line -match "RealEvaluatedCondition") {
                    Write-Host "Known error suppressed (RealEvaluatedCondition)"
                } elseif ($line -match "\[ERROR\]") {
                    Write-Host $line
                }
            }

            $proc.WaitForExit()
            if ($proc.ExitCode -ne 0) {
                throw "CubeMX exited with code $($proc.ExitCode)."
            }
            Write-Host "Generated $generated file(s)."
        }
        Remove-Item $batchScript -Force -ErrorAction SilentlyContinue
    }
}
