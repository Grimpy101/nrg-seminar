import math
import os
import subprocess
from pathlib import Path
from typing import TextIO

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from tap import Tap
from PIL import Image


class Vec3:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __sub__(self, other: 'Vec3') -> 'Vec3':
        x = self.x - other.x
        y = self.y - other.y
        z = self.z - other.z
        return Vec3(x, y, z)

    def __add__(self, other: 'Vec3') -> 'Vec3':
        x = self.x + other.x
        y = self.y + other.y
        z = self.z + other.z
        return Vec3(x, y, z)

    def __neg__(self) -> 'Vec3':
        x = -self.x
        y = -self.y
        z = -self.z
        return Vec3(x, y, z)

    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self):
        l = self.length()
        self.x = self.x / l
        self.y = self.y / l
        self.z = self.z / l

    def dot(self, other: 'Vec3') -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: 'Vec3') -> 'Vec3':
        x = self.y * other.z - self.z * other.y
        y = self.z * other.x - self.x * other.z
        z = self.x * other.y - self.y * other.x
        return Vec3(x, y, z)


class ArgumentParser(Tap):
    volume: str  # path to the volume file
    volume_resolution: list[int] = [256, 256, 256]  # three integers, representing volume resolution
    vpt: str = './local-vpt'  # path to the VPT renderer program
    output_resolution: int  # an integer representing width and height of output image
    output_path: str  # where to output data
    steps: int  # rendering quality per iteration (more means better quality image)
    iterations: int  # rendering iterations (more means better quality image)
    extinction: float  # extinction parameter for rendering
    focal_length: float = 2.0
    transfer_function: str  # path to binary transfer function file
    images: int  # amount of images to render

    def configure(self) -> None:
        self.add_argument('--volume_resolution', nargs=3)

    def process_args(self) -> None:
        volume_path = Path(self.volume)
        if not volume_path.exists():
            raise ValueError('Volume file does not exist')

        vpt_path = Path(self.vpt)
        if not vpt_path.exists():
            raise ValueError('VPT executable does not exist')

        transfer_function_path = Path(self.transfer_function)
        if not transfer_function_path.exists():
            raise ValueError('Transfer function file does not exist')


def look_at_quaternion(look_from: Vec3, look_to: Vec3, up: Vec3) -> tuple[float, float, float, float]:
    a = up
    b = look_to - look_from
    b.normalize()

    phi = -math.acos(a.dot(b))
    cos_hphi = math.cos(phi / 2)
    sin_hphi = math.sin(phi / 2)

    axis = a.cross(b)
    if axis.x == 0 and axis.y == 0 and axis.z == 0:
        if a.x == b.x and a.y == b.y and a.z == b.z:
            return 0, 0, 0, 1
    else:
        axis.normalize()

    x = axis.x * sin_hphi
    y = axis.y * sin_hphi
    z = axis.z * sin_hphi
    w = cos_hphi

    l = math.sqrt(x*x + y*y + z*z + w*w)
    x = x / l
    y = y / l
    z = z / l
    w = w / l
    return x, y, z, w


def quaternion_mult(q, r):
    return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]


def point_rotation_by_quaternion(point, q):
    r = [0]+point
    q_conj = [q[0],-1*q[1],-1*q[2],-1*q[3]]
    return quaternion_mult(quaternion_mult(q,r),q_conj)[1:]


def test_render(
        arguments: ArgumentParser,
        images_file: TextIO,
        ax,
):
    volume_res = arguments.volume_resolution

    #x1 = y1 = z1 = 1 / math.sqrt(3)
    x1 = 1
    y1 = 1
    z1 = 1

    x2 = 0
    y2 = 0
    z2 = 1

    (qx1, qy1, qz1, qw1) = look_at_quaternion(Vec3(x1, y1, z1), Vec3(0, 0, 0), Vec3(0, 0, -1))
    #print(qx1, qy1, qz1, qw1)

    #qx1 = 0.361
    #qy1 = 0.361
    #qz1 = -0
    #qw1 = 0.860

    (qx2, qy2, qz2, qw2) = look_at_quaternion(Vec3(-y2, x2, z2), Vec3(0, 0, 0), Vec3(0, 0, -1))

    # qx2 = 0
    # qy2 = 0
    # qz2 = -0.707
    # qw2 = 0.707

    output_file1 = f'{arguments.output_path}/raw_images/0.ppm'
    # subprocess.run([
    #     arguments.vpt,
    #     '--volume', arguments.volume,
    #     '--volume-dimensions', str(volume_res[0]), str(volume_res[1]), str(volume_res[2]),
    #     # '--tf', arguments.transfer_function,
    #     '--camera-position', str(x1), str(y1), str(z1),
    #     '--out-resolution', str(arguments.output_resolution), str(arguments.output_resolution),
    #     '--output', output_file1,
    #     '--steps', str(arguments.steps),
    #     '--iterations', str(arguments.iterations),
    #     '--extinction', str(arguments.extinction),
    #     '--focal-length', str(arguments.focal_length),
    #     '--linear'
    # ])
    #
    # image = Image.open(output_file1)
    # image.save(f"{arguments.output_path}/images/0.jpg")

    images_file.write(f'0 {qw1} {-qx1} {-qy1} {-qz1} {0} {0} {1} 1 0.jpg\n\n')

    output_file2 = f'{arguments.output_path}/raw_images/1.ppm'
    # subprocess.run([
    #     arguments.vpt,
    #     '--volume', arguments.volume,
    #     '--volume-dimensions', str(volume_res[0]), str(volume_res[1]), str(volume_res[2]),
    #     # '--tf', arguments.transfer_function,
    #     '--camera-position', str(x2), str(y2), str(z2),
    #     '--out-resolution', str(arguments.output_resolution), str(arguments.output_resolution),
    #     '--output', output_file2,
    #     '--steps', str(arguments.steps),
    #     '--iterations', str(arguments.iterations),
    #     '--extinction', str(arguments.extinction),
    #     '--focal-length', str(arguments.focal_length),
    #     '--linear'
    # ])
    #
    # image = Image.open(output_file2)
    # image.save(f"{arguments.output_path}/images/1.jpg")

    images_file.write(f'1 {qw2} {qx2} {qy2} {qz2} {0} {0} {-1} 1 1.jpg\n\n')


def render(
        arguments: ArgumentParser,
        images_file: TextIO,
        ax,
):
    images_amount = arguments.images
    volume_res = arguments.volume_resolution

    # Fibonacci sphere algorithm
    phi = math.pi * (math.sqrt(5.0) - 1.0)

    for i in range(images_amount):
        # Fibonacci sphere algorithm
        y = 1 - (i / float(images_amount - 1)) * 2
        radius = math.sqrt(1 - y * y)
        theta = phi * i

        x = math.cos(theta) * radius
        z = math.sin(theta) * radius

        output_file = f'{arguments.output_path}/raw_images/{i}.ppm'
        subprocess.run([
            arguments.vpt,
            '--volume', arguments.volume,
            '--volume-dimensions', str(volume_res[0]), str(volume_res[1]), str(volume_res[2]),
            # '--tf', arguments.transfer_function,
            '--camera-position', str(x), str(y), str(z),
            '--out-resolution', str(arguments.output_resolution), str(arguments.output_resolution),
            '--output', output_file,
            '--steps', str(arguments.steps),
            '--iterations', str(arguments.iterations),
            '--extinction', str(arguments.extinction),
            '--focal-length', str(arguments.focal_length),
            '--linear'
        ])

        image = Image.open(output_file)
        image.save(f"{arguments.output_path}/images/{i}.jpg")

        (qx, qy, qz, qw) = look_at_quaternion(Vec3(x, y, z), Vec3(0, 0, 0), Vec3(0, 0, -1))

        images_file.write(f'{i} {qw} {qx} {qy} {qz} {0} {0} {1} 1 {i}.jpg\n\n')


def main():
    args = ArgumentParser().parse_args()

    output_folder = Path(args.output_path)
    if not output_folder.exists():
        output_folder.mkdir(parents=True)
    images_folder = Path(f"{args.output_path}/images")
    if not images_folder.exists():
        images_folder.mkdir(parents=True)
    raw_image_folder = Path(f"{args.output_path}/raw_images")
    if not raw_image_folder.exists():
        raw_image_folder.mkdir(parents=True)
    sparse_folder = Path(f"{args.output_path}/sparse")
    if not sparse_folder.exists():
        sparse_folder.mkdir(parents=True)

    with open(f'{sparse_folder}/cameras.txt', 'w') as cameras_file:
        f = args.focal_length * args.output_resolution
        w2 = args.output_resolution / 2
        cameras_file.write(f'1 SIMPLE_PINHOLE {args.output_resolution} {args.output_resolution} {f} {w2} {w2}')

    with open(f'{sparse_folder}/points3D.txt', 'w'):
        pass

    with open(f'{sparse_folder}/images.txt', 'w') as images_file:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        render(args, images_file, ax)
        #plt.show()


main()
