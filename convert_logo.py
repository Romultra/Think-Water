#!/usr/bin/env python3
"""
Convert 176x176 PNG to compressed RGB565 C array for ESP32 PROGMEM storage
Only stores non-black pixels to save flash space
"""

from PIL import Image
import sys

def rgb888_to_rgb565(r, g, b):
    """Convert 24-bit RGB to 16-bit RGB565"""
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

def convert_png_to_header(png_path, header_path):
    """Convert PNG to compressed C header file with PROGMEM array"""

    # Open and verify image
    img = Image.open(png_path)
    if img.size != (176, 176):
        print(f"Error: Image must be 176x176, got {img.size}")
        sys.exit(1)

    # Convert to RGB if necessary
    if img.mode != 'RGB':
        img = img.convert('RGB')

    print(f"Converting {png_path} ({img.size[0]}x{img.size[1]})...")

    # Extract only non-black pixels
    # Format: x (uint8_t), y (uint8_t), color (uint16_t) = 4 bytes per pixel
    non_black_pixels = []
    total_pixels = 0

    for y in range(img.height):
        for x in range(img.width):
            r, g, b = img.getpixel((x, y))[:3]  # Get RGB, ignore alpha if present
            total_pixels += 1

            # Skip pure black pixels (optimize for #000000 background)
            if r == 0 and g == 0 and b == 0:
                continue

            rgb565 = rgb888_to_rgb565(r, g, b)
            non_black_pixels.append((x, y, rgb565))

    # Calculate compression ratio
    full_size = total_pixels * 2  # 2 bytes per pixel for full RGB565
    compressed_size = len(non_black_pixels) * 4  # 4 bytes per non-black pixel
    compression = (1 - compressed_size / full_size) * 100

    print(f"  Total pixels: {total_pixels}")
    print(f"  Non-black pixels: {len(non_black_pixels)} ({len(non_black_pixels)/total_pixels*100:.1f}%)")
    print(f"  Full size: {full_size} bytes ({full_size / 1024:.1f} KB)")
    print(f"  Compressed size: {compressed_size} bytes ({compressed_size / 1024:.1f} KB)")
    print(f"  Space saved: {compression:.1f}%")

    # Write C header file
    with open(header_path, 'w') as f:
        f.write("// Auto-generated boot logo - 176x176 RGB565 (compressed)\n")
        f.write("// Background is black (#000000), only non-black pixels stored\n")
        f.write("// Generated from: " + png_path + "\n")
        f.write("#ifndef BOOT_LOGO_H\n")
        f.write("#define BOOT_LOGO_H\n\n")
        f.write("#include <Arduino.h>\n\n")
        f.write("// Boot logo pixel data: x, y, color (RGB565)\n")
        f.write(f"// Non-black pixels: {len(non_black_pixels)} / {total_pixels} ({len(non_black_pixels)/total_pixels*100:.1f}%)\n")
        f.write(f"// Storage: {compressed_size} bytes ({compressed_size / 1024:.1f} KB) - {compression:.1f}% smaller than full image\n")
        f.write(f"#define BOOT_LOGO_PIXEL_COUNT {len(non_black_pixels)}\n\n")

        # Struct definition
        f.write("struct BootLogoPixel {\n")
        f.write("    uint8_t x;\n")
        f.write("    uint8_t y;\n")
        f.write("    uint16_t color;\n")
        f.write("} __attribute__((packed));\n\n")

        # Pixel data array
        f.write("const BootLogoPixel BOOT_LOGO_DATA[] PROGMEM = {\n")

        # Write pixels (6 per line for readability)
        for i in range(0, len(non_black_pixels), 6):
            row = non_black_pixels[i:i+6]
            f.write("    ")
            f.write(", ".join(f"{{{p[0]}, {p[1]}, 0x{p[2]:04X}}}" for p in row))
            if i + 6 < len(non_black_pixels):
                f.write(",")
            f.write("\n")

        f.write("};\n\n")
        f.write("#endif // BOOT_LOGO_H\n")

    print(f"\n[OK] Generated {header_path}")

if __name__ == "__main__":
    convert_png_to_header("Boot logo.png", "src/base/boot_logo.h")
