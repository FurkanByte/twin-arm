"""freecad/drawings/*.pdf -> freecad/TwinArm5_Drawings.pdf (in page order)."""
from pathlib import Path

from pypdf import PdfWriter

HERE = Path(__file__).resolve().parent
files = sorted((HERE / "drawings").glob("*.pdf"))
w = PdfWriter()
for f in files:
    w.append(str(f))
out = HERE / "TwinArm5_Drawings.pdf"
with open(out, "wb") as fh:
    w.write(fh)
print(f"{len(files)} pages -> {out}")
