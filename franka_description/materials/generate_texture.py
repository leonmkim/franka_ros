#%%
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from noise import pnoise2

#%%
height,width = 512,512
# randomly generate a base color
# base_color = np.random.randint(0, 256, 3, dtype=np.uint8)
base_color = np.ones(3)*255
pattern_intensity = 0.5
frequency=0.2,
angle=np.pi/4
# generate a png texture file with a base color and a random noise pattern on top
# Create an array with the base color
base_array = np.full((height, width, 3), base_color, dtype=np.uint8)
plt.imshow(base_array)
#%%
# Generate grayscale perline noise
noise_array = np.zeros((height, width, 1), dtype=np.uint8)
for i in range(height):
    for j in range(width):
        noise_array[i,j] = int(pnoise2(i/100, j/100, octaves=6, persistence=1.0, lacunarity=2.0, repeatx=512, repeaty=512, base=0)*128+128)

# Generate a random grayscale pattern
# pattern_array = np.random.randint(0, 256, (height, width,1), dtype=np.uint8)
# Generate a wave pattern
# Create a coordinate grid
# y = np.linspace(0, height, height)
# x = np.linspace(0, width, width)
# X, Y = np.meshgrid(x, y)
# wave_pattern = np.sin((X * np.cos(angle) + Y * np.sin(angle)) * frequency)

# # Normalize the wave pattern to 0-255 and convert to grayscale (same value for R, G, and B)
# wave_pattern_normalized = np.interp(wave_pattern, (wave_pattern.min(), wave_pattern.max()), (0, 255))
# wave_array = np.repeat(wave_pattern_normalized[:, :, np.newaxis], 3, axis=2).astype(np.uint8)
plt.imshow(noise_array)
# show the colorbar
plt.colorbar()
#%%
# Blend the base color with the random pattern
# Adjust pattern_intensity to control the visibility of the pattern
textured_array = np.clip((1-pattern_intensity)*base_array + pattern_intensity * (noise_array ), 0, 255).astype(np.uint8)

# Create an image from the array
texture_image = Image.fromarray(textured_array)
plt.imshow(texture_image)
# %%
# Save the image to a file
filename = 'random2'
texture_image.save(f'{filename}.png')
# %%
