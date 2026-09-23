# Annotation rules

Masks must be single-channel uint8 PNG. Use 255 only for pixels that should not contribute to loss. Keep RGB, mask, and optional depth-reference stems identical. Depth is annotation reference only and must not be supplied to the model.

Annotate the fine eight-class taxonomy first. Generate coarse masks using `convert_fine_to_coarse.py`; never edit ignore pixels into class 0. Split temporally adjacent rover frames as groups to avoid train/validation leakage.
