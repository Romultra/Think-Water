import re

Import("env") # type: ignore

env_name = env["PIOENV"] # type: ignore

with open('wokwi.toml', 'r', encoding='utf-8') as f:
    content = f.read()

# Update wokwi.toml firmware and elf paths to use the given env
with open('wokwi.toml', 'r', encoding='utf-8') as f:
    content = f.read()

# Replace the env part in the firmware and elf paths
content = re.sub(
    r"\.pio/build/[^/\\]+/firmware\.bin",
    f".pio/build/{env_name}/firmware.bin",
    content
)
content = re.sub(
    r"\.pio/build/[^/\\]+/firmware\.elf",
    f".pio/build/{env_name}/firmware.elf",
    content
)

with open('wokwi.toml', 'w', encoding='utf-8') as f:
    f.write(content)

print(f"wokwi.toml updated with environment: {env_name}")