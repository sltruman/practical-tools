import imageio.v3 as iio
from wgpu.gui.auto import WgpuCanvas, run
import pygfx as gfx


# Init
canvas = WgpuCanvas(size=(640, 480), title="pbr测试")
renderer = gfx.renderers.WgpuRenderer(canvas)
scene = gfx.Scene()
# scene.world.up = (0,0,1)

# Read cube image and turn it into a 3D image (a 4d array)
env_img = iio.imread("imageio:meadow_cube.jpg")
cube_size = env_img.shape[1]
env_img.shape = 6, cube_size, cube_size, env_img.shape[-1]

# Create environment map
env_tex = gfx.Texture(env_img, dim=2, size=(cube_size, cube_size, 6), generate_mipmaps=True)

# Apply env map to skybox
background = gfx.Background(None, gfx.BackgroundSkyboxMaterial(map=env_tex))
background.world.up=(0,0,1)

scene.add(background)

# Load meshes, and apply env map
# Note that this lights the helmet already
gltf_path = "C:/Users/SLTru/Desktop/practical-tools/practools/store/env/Room/visual.glb"
meshes = gfx.load_meshes(gltf_path)

for m in meshes:
    m.geometry.texcoords1 = m.geometry.texcoords  # use second set of texcoords for ao map
    m.material.env_map = env_tex

scene.add(*meshes)

# Add extra light more or less where the sun seems to be in the skybox
# light = gfx.SpotLight(color="#ffffff")
# light.local.position = (-5, -1, 1)
# scene.add(light)

# Create camera and controller
camera = gfx.PerspectiveCamera(45, 640 / 480)
camera.show_object(scene, view_dir=(1.8, -0.6, -2.7),up=(0,0,1))
controller = gfx.OrbitController(camera, register_events=renderer)

if __name__ == "__main__":
    renderer.request_draw(lambda: renderer.render(scene, camera))
    run()