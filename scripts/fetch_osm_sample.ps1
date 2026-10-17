<#
.SYNOPSIS
  Download a Warsaw OSM Overpass sample (small regression or ~1 km larger extract).

.DESCRIPTION
  Fetches highway ways + referenced nodes + type=restriction relations for a
  rectangular bounding box. Writes only after confirming valid Overpass JSON
  with elements. Reuses an existing valid output unless -Force is set.
  Never overwrites a good sample with error HTML/JSON.

.PARAMETER Sample
  Named preset: Small (default, ~220 m) or Larger (~1 km). Sets OutFile + bbox.

.PARAMETER OutFile
  Destination path for the Overpass JSON sample (overrides Sample default path).

.PARAMETER BBox
  Optional south,west,north,east override (comma-separated). When set with -Sample,
  still uses that sample's default OutFile unless -OutFile is also given.

.PARAMETER Force
  Re-download even if OutFile already exists and looks valid.

.PARAMETER TimeoutSec
  HTTP / Overpass client timeout in seconds (default 90 for Larger, 60 for Small).

.EXAMPLE
  .\scripts\fetch_osm_sample.ps1
  .\scripts\fetch_osm_sample.ps1 -Sample Larger
  .\scripts\fetch_osm_sample.ps1 -Sample Larger -Force
  .\scripts\fetch_osm_sample.ps1 -OutFile data\osm\warsaw_sample.overpass.json
#>
[CmdletBinding()]
param(
    [ValidateSet("Small", "Larger")]
    [string]$Sample = "Small",
    [string]$OutFile = "",
    [string]$BBox = "",
    [switch]$Force,
    [int]$TimeoutSec = 0
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$scriptDir = $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($scriptDir)) {
    $scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
}
$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $scriptDir ".."))

# Presets — Small keeps Stage 1–3 regression; Larger is Stage 4 credible checks (~1 km).
# Both centered near ul. Wilcza / ul. Krucza (Warsaw Śródmieście). NOT whole city.
$presets = @{
    Small = @{
        OutRel     = "data\osm\warsaw_sample.overpass.json"
        BBoxSouth  = "52.22480"
        BBoxWest   = "21.01580"
        BBoxNorth  = "52.22680"
        BBoxEast   = "21.01900"
        TimeoutSec = 60
        Label      = "small (~220 m)"
    }
    Larger = @{
        OutRel     = "data\osm\warsaw_1km.overpass.json"
        # ~1.0 km N-S × ~1.0 km E-W around the same center as Small.
        BBoxSouth  = "52.22130"
        BBoxWest   = "21.01040"
        BBoxNorth  = "52.23030"
        BBoxEast   = "21.02440"
        TimeoutSec = 90
        Label      = "larger (~1 km)"
    }
}

$preset = $presets[$Sample]

if ([string]::IsNullOrWhiteSpace($OutFile)) {
    $OutFile = Join-Path $repoRoot $preset.OutRel
}

if ($TimeoutSec -le 0) {
    $TimeoutSec = [int]$preset.TimeoutSec
}

if ([string]::IsNullOrWhiteSpace($BBox)) {
    $BBox = "$($preset.BBoxSouth),$($preset.BBoxWest),$($preset.BBoxNorth),$($preset.BBoxEast)"
}

# Explicit Overpass endpoint (public instance). Same default as in-app MapDownloadService.
$OverpassUrl = "https://overpass-api.de/api/interpreter"

# Identifying User-Agent — keep in sync with osm_net::kEasyRiderUserAgent (C++).
$UserAgent = "EasyRider/0.1 (City Traffic Simulator; educational; local desktop)"

# Highway ways + member nodes + type=restriction relations that reference those ways.
# Small preset stays in sync with osm_net::buildRoadNetworkQuery (src/osm/OverpassQuery.cpp).
# Larger also pulls tagged crossing / signal / stop / give_way nodes (skel omits tags).
$queryTimeout = [Math]::Max(25, [Math]::Min(120, $TimeoutSec - 10))
if ($Sample -eq "Larger") {
    $Query = @"
[out:json][timeout:$queryTimeout];
(
  way["highway"~"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|service)$"]($BBox);
)->.roads;
.roads out body;
.roads >;
out skel qt;
(
  node["highway"~"^(crossing|traffic_signals|stop|give_way)$"]($BBox);
);
out body;
rel(bw.roads)["type"="restriction"];
out body;
"@
} else {
    $Query = @"
[out:json][timeout:$queryTimeout];
(
  way["highway"~"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|service)$"]($BBox);
)->.roads;
.roads out body;
.roads >;
out skel qt;
rel(bw.roads)["type"="restriction"];
out body;
"@
}

function Test-OverpassJsonFile {
    param([Parameter(Mandatory)][string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) {
        return $false
    }

    try {
        $raw = Get-Content -LiteralPath $Path -Raw -ErrorAction Stop
        if ([string]::IsNullOrWhiteSpace($raw)) {
            return $false
        }
        # Reject obvious HTML error pages before ConvertFrom-Json.
        $trimmed = $raw.TrimStart()
        if ($trimmed.StartsWith("<") -or $trimmed -match "(?i)^\s*<!DOCTYPE\s+html") {
            return $false
        }

        $obj = $raw | ConvertFrom-Json -ErrorAction Stop
        if ($null -eq $obj) {
            return $false
        }
        if (-not ($obj.PSObject.Properties.Name -contains "elements")) {
            return $false
        }
        if ($null -eq $obj.elements) {
            return $false
        }
        # Overpass success payloads always include elements (may be empty array).
        # Require at least one element for a usable road sample.
        if (@($obj.elements).Count -lt 1) {
            return $false
        }
        return $true
    }
    catch {
        return $false
    }
}

function Write-Info {
    param([string]$Message)
    Write-Host "[fetch_osm_sample] $Message"
}

$OutFile = [System.IO.Path]::GetFullPath($OutFile)
$outDir = Split-Path -Parent $OutFile
if (-not (Test-Path -LiteralPath $outDir)) {
    New-Item -ItemType Directory -Force -Path $outDir | Out-Null
}

if (-not $Force -and (Test-OverpassJsonFile -Path $OutFile)) {
    Write-Info "Reusing existing valid sample ($($preset.Label)): $OutFile"
    Write-Info "Pass -Force to re-download."
    exit 0
}

$tempFile = Join-Path $outDir ("." + [System.IO.Path]::GetFileName($OutFile) + ".tmp")
if (Test-Path -LiteralPath $tempFile) {
    Remove-Item -LiteralPath $tempFile -Force
}

Write-Info "Sample: $Sample ($($preset.Label))"
Write-Info "Endpoint: $OverpassUrl"
Write-Info "BBox (S,W,N,E): $BBox"
Write-Info "TimeoutSec: $TimeoutSec"
Write-Info "Downloading to temp: $tempFile"

try {
    # Single attempt — no parallel requests, no unlimited retries.
    # Overpass expects form field "data" with the QL query.
    # Public Overpass instances require an identifying User-Agent (406 otherwise).
    $postBody = "data=" + [System.Uri]::EscapeDataString($Query)
    $headers = @{
        "User-Agent" = $UserAgent
        "Accept"     = "application/json"
    }
    $response = Invoke-WebRequest `
        -Uri $OverpassUrl `
        -Method Post `
        -Body $postBody `
        -Headers $headers `
        -ContentType "application/x-www-form-urlencoded; charset=UTF-8" `
        -TimeoutSec $TimeoutSec `
        -UseBasicParsing

    if ($null -eq $response) {
        throw "Empty HTTP response."
    }
    if ($response.StatusCode -lt 200 -or $response.StatusCode -ge 300) {
        throw "HTTP $($response.StatusCode): unexpected status."
    }

    $content = $response.Content
    if ([string]::IsNullOrWhiteSpace($content)) {
        throw "HTTP response body is empty."
    }

    # Persist temp first; validate before replacing the final file.
    [System.IO.File]::WriteAllText($tempFile, $content, [System.Text.UTF8Encoding]::new($false))

    if (-not (Test-OverpassJsonFile -Path $tempFile)) {
        throw "Downloaded payload is not valid Overpass JSON with elements (possible error page or empty result)."
    }

    # Atomic-ish replace: only now touch the final path.
    if (Test-Path -LiteralPath $OutFile) {
        Remove-Item -LiteralPath $OutFile -Force
    }
    Move-Item -LiteralPath $tempFile -Destination $OutFile -Force

    $size = (Get-Item -LiteralPath $OutFile).Length
    $obj = (Get-Content -LiteralPath $OutFile -Raw) | ConvertFrom-Json
    $ways = @($obj.elements | Where-Object { $_.type -eq "way" }).Count
    $nodes = @($obj.elements | Where-Object { $_.type -eq "node" }).Count
    $rels = @($obj.elements | Where-Object { $_.type -eq "relation" }).Count
    Write-Info "Wrote valid sample ($size bytes): $OutFile"
    Write-Info "elements: ways=$ways nodes=$nodes relations=$rels"
    exit 0
}
catch {
    Write-Error ("Fetch failed: {0}" -f $_.Exception.Message)
    if (Test-Path -LiteralPath $tempFile) {
        Remove-Item -LiteralPath $tempFile -Force -ErrorAction SilentlyContinue
    }
    # Preserve any previously good final file — do not overwrite with failures.
    if (Test-OverpassJsonFile -Path $OutFile) {
        Write-Info "Previous valid sample left unchanged: $OutFile"
    }
    else {
        Write-Info "No valid sample at: $OutFile"
        Write-Info "Mark as MISSING until a successful fetch."
        Write-Info "Command: .\scripts\fetch_osm_sample.ps1 -Sample $Sample"
    }
    exit 1
}
