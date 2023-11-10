import imageio.v3 as iio
from wgpu.gui.auto import WgpuCanvas, run
import pygfx as gfx


# Init
canvas = WgpuCanvas(size=(640, 480), title="gfx_pbr")
renderer = gfx.renderers.WgpuRenderer(canvas)
scene = gfx.Scene()

# Read cube image and turn it into a 3D image (a 4d array)
env_img = iio.imread("imageio:meadow_cube.jpg")
cube_size = env_img.shape[1]
env_img.shape = 6, cube_size, cube_size, env_img.shape[-1]

# Create environment map
env_tex = gfx.Texture(
    env_img, dim=2, size=(cube_size, cube_size, 6), generate_mipmaps=True
)

# Apply env map to skybox
background = gfx.Background(None, gfx.BackgroundSkyboxMaterial(map=env_tex))
scene.add(background)

# Load meshes, and apply env map
# Note that this lits the helmet already
gltf_path = 'data/DamagedHelmet/glTF/DamagedHelmet.gltf'
meshes = gfx.load_meshes(gltf_path)
scene.add(*meshes)
m = meshes[0]  # this example has just one mesh
m.geometry.texcoords1 = m.geometry.texcoords  # use second set of texcoords for ao map
m.material.env_map = env_tex

# Add extra light more or less where the sun seems to be in the skybox
light = gfx.SpotLight(color="#444")
light.local.position = (-500, 1000, -1000)
scene.add(light)

# Create camera and controller
camera = gfx.PerspectiveCamera(45, 640 / 480)
camera.show_object(m, view_dir=(1.8, -0.6, -2.7))
controller = gfx.OrbitController(camera, register_events=renderer)


if __name__ == "__main__":
    renderer.request_draw(lambda: renderer.render(scene, camera))
    run()