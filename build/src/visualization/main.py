import animation
import config

print("Generating the animation......It may take a few time.")
print("If you want to generate faster with low quality, you can modify in config.py.")
ani = animation.Animation(config.file_path)
ani.save()
print("Done.")
