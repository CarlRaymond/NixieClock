param($height)
$inkscape = "C:\Program files\Inkscape\Inkscape.exe"
$magick = "C:\Program Files\ImageMagick-7.0.10-Q16\magick.exe"

$destination = Join-Path $(Get-Location) $height
New-Item -Path $destination -ItemType Directory -Force | Out-Null

$all = New-Object System.Collections.ArrayList
Get-ChildItem -Filter "*.svg" | Foreach-Object {
    $name_png = $([io.path]::ChangeExtension($_.Name, 'png' ))
    $name_bmp = $([io.path]::ChangeExtension($_.Name, 'bmp' ))
    Start-Process -Wait -FilePath $inkscape @('-z', '-b #3186f500', "-h $height", "-e $name_png", "$($_.FullName)")
    & $magick @('convert', $([io.path]::ChangeExtension($_.Name, 'png' )), "BMP3:$name_bmp")
    Move-Item -Destination $destination -Path $name_bmp -Force
    $all.Add($name_png) | Out-Null
}

Write-Output 'All items: ' $all
& $magick convert -delay 50 $all -loop 0 loop.gif
