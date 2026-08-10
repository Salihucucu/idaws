# LiDAR taramasından ayrıştırılmış tek bir engel kümesi.
# Koordinatlar araç gövde çerçevesinde: +x ileri, +y sol (metre).
int32 id
float32 center_x
float32 center_y
float32 range          # kümenin merkezine mesafe (m)
float32 bearing        # kümenin merkezinin kerteriz açısı (rad, +x'ten CCW)
float32 min_range      # kümedeki en yakın ışının mesafesi (m)
float32 width          # kümenin yaklaşık genişliği (m)
int32 point_count
