$bal = 0
$line=0
$maxBal = 0
$maxLine = 0
Get-Content src\main.cpp | ForEach-Object {
    $line++
    $bal += (($_ -split "\\{").Count - 1) - (($_ -split "\\}").Count - 1)
    if ($bal -lt 0) { Write-Host ("NEGATIVE at line " + $line + ": bal=" + $bal); exit }
    if ($bal -gt $maxBal) { $maxBal = $bal; $maxLine = $line }
}
Write-Host ("FINAL BAL=" + $bal)
Write-Host ("MAX BAL=" + $maxBal + " at line " + $maxLine)
