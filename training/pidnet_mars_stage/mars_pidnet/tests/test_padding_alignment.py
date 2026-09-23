import numpy as np
from mars_pidnet.datasets import pad_640x360

def test_padding_preserves_content_and_alignment():
    image=np.zeros((360,640,3),np.uint8); mask=np.zeros((360,640),np.uint8)
    image[123,321]=[7,8,9]; mask[123,321]=4
    out, lab=pad_640x360(image,mask)
    assert out.shape==(384,640,3) and lab.shape==(384,640)
    assert np.array_equal(out[135,321],[7,8,9]) and lab[135,321]==4
    assert np.all(lab[:12]==255) and np.all(lab[-12:]==255)
    assert np.array_equal(out[12:372],image) and np.array_equal(lab[12:372],mask)
