$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$build = Join-Path $repo "build-host"
cmake -S (Join-Path $repo "test") -B $build -G "MinGW Makefiles"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cmake --build $build --parallel
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
ctest --test-dir $build --output-on-failure
exit $LASTEXITCODE
