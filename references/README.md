# Local vendor references

`official/` may contain the vendor PDF bundle used during read-only audits. The
PDFs are large binary source material and are intentionally ignored rather than
stored in Git. Their filenames, metadata, priorities and M1 relevance are recorded
in `docs/reference_inventory.md`.

Do not treat a local PDF as runtime authority when it conflicts with the deployed
Jetson source or a recorded observation.
