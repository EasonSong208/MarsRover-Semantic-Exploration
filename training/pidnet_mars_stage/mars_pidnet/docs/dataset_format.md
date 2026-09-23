# Mars dataset format

RGB files are `.png`, `.jpg`, or `.jpeg`; masks are single-channel mode-L `uint8` PNG files with the same stem. Fine masks allow `0..7,255`; coarse masks allow `0..5,255`. The deterministic conversion is `5,6,7 -> 5`, and `255` remains ignored.

Each `splits/{train,val,test}.txt` contains one stem per line. Files live below `images/<split>`, `masks_fine/<split>`, and `masks_coarse/<split>`. Depth is reference-only and is never loaded by this baseline.

Every 640x360 image/mask pair is padded at the top and bottom by 12 pixels to 640x384. RGB uses value 0; masks use 255. No resizing is performed. Horizontal flipping is applied jointly to RGB and mask.

Run `../tools/validate_dataset.py DATASET --granularity coarse --report-dir REPORTS` before training. Adjacent-frame detection is a filename-number heuristic and therefore reports warnings for manual review.
