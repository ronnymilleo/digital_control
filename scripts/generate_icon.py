#!/usr/bin/env python3
"""
Generate a sample icon for the Digital Control application.
Creates a simple icon with a control system theme.
"""

from PIL import Image, ImageDraw, ImageFont
import os

def create_icon():
    # Create a new image with a gradient background
    size = 256
    img = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    
    # Create gradient background
    for i in range(size):
        color_intensity = int(20 + (i / size) * 60)
        draw.rectangle([(0, i), (size, i+1)], 
                      fill=(color_intensity, color_intensity + 20, color_intensity + 40, 255))
    
    # Draw a control system block diagram representation
    # Main block
    block_size = 80
    block_x = size // 2 - block_size // 2
    block_y = size // 2 - block_size // 2
    
    # Draw main control block
    draw.rounded_rectangle(
        [(block_x, block_y), (block_x + block_size, block_y + block_size)],
        radius=10,
        fill=(50, 150, 250, 255),
        outline=(255, 255, 255, 255),
        width=3
    )
    
    # Draw "C" for Controller in the center
    try:
        # Try to use a font, fall back to default if not available
        from PIL import ImageFont
        font = ImageFont.truetype("arial.ttf", 60)
    except:
        font = ImageFont.load_default()
    
    text = "C"
    bbox = draw.textbbox((0, 0), text, font=font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    text_x = size // 2 - text_width // 2
    text_y = size // 2 - text_height // 2 - 5
    draw.text((text_x, text_y), text, fill=(255, 255, 255, 255), font=font)
    
    # Draw input arrow
    arrow_length = 50
    arrow_y = size // 2
    draw.line([(20, arrow_y), (block_x - 5, arrow_y)], fill=(255, 255, 255, 255), width=3)
    draw.polygon([(block_x - 5, arrow_y - 8), 
                  (block_x - 5, arrow_y + 8), 
                  (block_x + 5, arrow_y)], 
                 fill=(255, 255, 255, 255))
    
    # Draw output arrow
    draw.line([(block_x + block_size + 5, arrow_y), (size - 20, arrow_y)], 
              fill=(255, 255, 255, 255), width=3)
    draw.polygon([(size - 20, arrow_y - 8), 
                  (size - 20, arrow_y + 8), 
                  (size - 10, arrow_y)], 
                 fill=(255, 255, 255, 255))
    
    # Draw feedback loop
    feedback_y_top = block_y - 20
    feedback_y_bottom = block_y + block_size + 20
    draw.line([(block_x + block_size - 20, block_y + block_size), 
               (block_x + block_size - 20, feedback_y_bottom)], 
              fill=(200, 200, 200, 255), width=2)
    draw.line([(block_x + block_size - 20, feedback_y_bottom), 
               (block_x + 20, feedback_y_bottom)], 
              fill=(200, 200, 200, 255), width=2)
    draw.line([(block_x + 20, feedback_y_bottom), 
               (block_x + 20, block_y + block_size)], 
              fill=(200, 200, 200, 255), width=2)
    
    return img

def main():
    # Create assets directory if it doesn't exist
    assets_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'assets')
    os.makedirs(assets_dir, exist_ok=True)
    
    # Generate and save icon
    icon = create_icon()
    icon_path = os.path.join(assets_dir, 'icon.png')
    icon.save(icon_path, 'PNG')
    print(f"Icon generated successfully at: {icon_path}")
    
    # Also create smaller versions for different uses
    for size in [64, 48, 32, 16]:
        small_icon = icon.resize((size, size), Image.Resampling.LANCZOS)
        small_path = os.path.join(assets_dir, f'icon_{size}.png')
        small_icon.save(small_path, 'PNG')
        print(f"Generated {size}x{size} icon at: {small_path}")

if __name__ == "__main__":
    main()
