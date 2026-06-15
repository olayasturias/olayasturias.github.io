$j = Get-Content assets/json/topic_papers.json -Raw | ConvertFrom-Json
$papers = $j.PSObject.Properties | ForEach-Object { $_.Value | Add-Member -NotePropertyName id -NotePropertyValue $_.Name -PassThru }

function Find($label, $pattern, [switch]$InAbstract, [switch]$InAuthors) {
    Write-Host "`n######## $label"
    $hits = $papers | Where-Object {
        $_.title -match $pattern -or
        ($InAbstract -and $_.abstract -match $pattern) -or
        ($InAuthors -and (($_.authors -join '; ') -match $pattern))
    }
    foreach ($h in $hits | Select-Object -First 4) {
        Write-Host "TITLE: $($h.title)"
        Write-Host "AUTHORS: $($h.authors -join ', ')"
        Write-Host "SRC: $($h.source) $($h.workshop)  URL: $($h.url)"
        $a = $h.abstract; if ($a.Length -gt 450) { $a = $a.Substring(0,450) + '...' }
        Write-Host "ABS: $a"
        Write-Host "---"
    }
    if (-not $hits) { Write-Host "(no match)" }
}

Find 'Jungseok Hong (author)' 'Jungseok' -InAuthors
Find 'Mangelson (author)' 'Mangelson' -InAuthors
Find 'own poster: Olaya' 'Olaya|Tu.{1,2}n' -InAuthors
Find 'own poster: monocular VO abstract' 'monocular visual odometry' -InAbstract
Find 'IGN/FAST-LIO poster' 'FAST-LIO|floor-?plan prior|geoportal|Instituto Geogr' -InAbstract
Find 'Symmetries (title)' 'Symmetr'
Find 'Jaquier (author)' 'Jaquier' -InAuthors
Find 'Why domain matters (verify)' 'Why Domain Matters'
Find 'ALAR (verify)' '\bALAR\b'
Find 'underwater Gaussian splatting' 'Underwater.*(Splat|Gaussian)|Gaussian.*Underwater'
Find 'Reproducibility' 'Reproducib'
Find 'VGGT (floor notes)' 'VGGT'
Find 'Map Anything' 'Map ?Anything'
