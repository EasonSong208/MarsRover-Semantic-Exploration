from pathlib import Path
import numpy as np
from PIL import Image
from mars_pidnet.datasets import MarsDataset

def test_dataset_pairing(tmp_path: Path):
    (tmp_path/"images/train").mkdir(parents=True); (tmp_path/"masks_coarse/train").mkdir(parents=True); (tmp_path/"splits").mkdir()
    Image.fromarray(np.zeros((360,640,3),np.uint8)).save(tmp_path/"images/train/a.jpg")
    Image.fromarray(np.zeros((360,640),np.uint8)).save(tmp_path/"masks_coarse/train/a.png")
    (tmp_path/"splits/train.txt").write_text("a\n")
    item=MarsDataset(tmp_path)[0]; assert item["image"].shape==(3,384,640); assert item["mask"].shape==(384,640)
