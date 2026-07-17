# Puzzle Piece Analysis Pipeline

## Overview
This pipeline extracts individual puzzle pieces from an image (white pieces with black line borders) and generates feature descriptors suitable for PCA analysis.

## Workflow

### 1. Piece Detection & Feature Extraction

The image processing pipeline:
- **Threshold**: Separates white puzzle pieces from black borders using grayscale threshold
- **Morphology**: Cleans up borders with closing operations
- **Contour Detection**: Finds individual piece boundaries
- **Feature Computation**: Extracts geometric/shape descriptors for each piece

### 2. Features Extracted Per Piece

Each puzzle piece gets a descriptor with:

| Feature | Description | Use Case |
|---------|-------------|----------|
| `area` | Pixel count of piece | Size-based clustering |
| `perimeter` | Boundary length | Complexity measure |
| `compactness` | Circularity (0-1) | Shape regularity |
| `eccentricity` | Ellipse elongation (0-1) | Orientation measure |
| `aspect_ratio` | Width/height ratio | Bounding shape |
| `solidity` | Area/hull_area (0-1) | Concavity measure |
| `hu_moments[7]` | Scale/rotation invariant descriptors | Robust shape matching |
| `center` | (x, y) centroid | Spatial position |
| `bbox` | Bounding rectangle | Quick bounds |
| `rotation` | Fitted ellipse angle | Orientation |

### 3. PCA Analysis

The feature matrix (N_pieces × 6 features) is:
1. **Standardized** using StandardScaler (zero mean, unit variance)
2. **Projected** onto principal components
3. **Visualized** showing piece distribution in lower-dimensional space

Colors in PCA plot represent piece area for additional context.

## Usage

### Command Line

```bash
# Basic analysis with 2D PCA projection
python src/solution_blob_finder.py media/puzzle.png

# With custom PCA components
python src/solution_blob_finder.py media/puzzle.png --components 3

# Save piece data to JSON
python src/solution_blob_finder.py media/puzzle.png --output-json pieces.json
```

### Python API

```python
from computer_vision.get_puzzle_solution import chunk_pieces, detect_puzzle_pieces
from sklearn.decomposition import PCA

# Extract pieces and features
pieces, feature_matrix = chunk_pieces("media/puzzle.png")

# Use feature matrix for PCA
from sklearn.preprocessing import StandardScaler
scaler = StandardScaler()
features_scaled = scaler.fit_transform(feature_matrix)

pca = PCA(n_components=2)
pca_result = pca.fit_transform(features_scaled)

# Access individual piece info
for piece in pieces:
    print(f"Piece {piece['piece_id']}: area={piece['area']}, shape={piece['hu_moments']}")
```

## Example: Clustering Similar Pieces

```python
from sklearn.cluster import KMeans

pieces, features = chunk_pieces("puzzle.png")
kmeans = KMeans(n_clusters=3)
clusters = kmeans.fit_predict(features)

for piece, cluster_id in zip(pieces, clusters):
    print(f"Piece {piece['piece_id']} -> Cluster {cluster_id}")
```

## Image Requirements

- **Format**: PNG or JPG
- **Content**: White puzzle pieces on any background
- **Borders**: Black lines separating pieces (minimum 2px wide)
- **Example**:
  ```
  ┌─────────────┐┌─────────────┐
  │   Piece 1   ││   Piece 2   │
  │  (white)    ││  (white)    │
  └─────────────┘└─────────────┘
   (black border lines)
  ```

## Output Files

- `puzzle_pca.png` - 2D PCA projection visualization
- `pieces.json` - Complete piece data with all descriptors (with `--output-json`)

## Dependencies

See `requirements.txt`:
- `numpy` - Array operations
- `opencv-python` - Image processing
- `scikit-learn` - PCA, clustering, scaling
- `matplotlib` - Visualization
