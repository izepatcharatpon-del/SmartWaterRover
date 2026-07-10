$dir = Split-Path -Parent $MyInvocation.MyCommand.Path
Start-Process (Join-Path $dir "index.html")
