import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
import mmap
from typing import Optional

class ImageGraySubscriber(Node):
    def __init__(self):
        super().__init__('image_gray_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # BRAM device paths and size for 88x88 mono8
        self.bram_write_path = '/dev/uio0'
        self.bram_read_path = '/dev/uio1'
        self.bram_size = 88 * 88  # bytes

        # Publisher for the image read back from BRAM1
        self.pub = self.create_publisher(Image, 'image_from_bram', 10)

        self.get_logger().info('Subscribed to /image_raw and publishing /image_from_bram')

    def _write_to_bram(self, img_bytes: bytes) -> bool:
        try:
            fd = os.open(self.bram_write_path, os.O_RDWR | os.O_SYNC)
        except Exception as e:
            self.get_logger().error(f'Failed to open BRAM write device {self.bram_write_path}: {e}')
            return False

        try:
            mem = mmap.mmap(fd, self.bram_size, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)
            mem.seek(0)
            if len(img_bytes) < self.bram_size:
                img_bytes = img_bytes.ljust(self.bram_size, b'\x00')
            mem.write(img_bytes[:self.bram_size])
            mem.flush()
            mem.close()
            os.close(fd)
            return True
        except Exception as e:
            self.get_logger().error(f'BRAM write error: {e}')
            try:
                mem.close()
            except Exception:
                pass
            try:
                os.close(fd)
            except Exception:
                pass
            return False

    def _read_from_bram(self) -> Optional[np.ndarray]:
        try:
            fd = os.open(self.bram_read_path, os.O_RDONLY | os.O_SYNC)
        except Exception as e:
            self.get_logger().error(f'Failed to open BRAM read device {self.bram_read_path}: {e}')
            return None

        try:
            mem = mmap.mmap(fd, self.bram_size, mmap.MAP_SHARED, mmap.PROT_READ)
            mem.seek(0)
            data = mem.read(self.bram_size)
            mem.close()
            os.close(fd)
            arr = np.frombuffer(data, dtype=np.uint8).reshape((88, 88)).copy()
            return arr
        except Exception as e:
            self.get_logger().error(f'BRAM read error: {e}')
            try:
                mem.close()
            except Exception:
                pass
            try:
                os.close(fd)
            except Exception:
                pass
            return None

    def _msg_to_gray(self, msg: Image) -> Optional[np.ndarray]:
        enc = (msg.encoding or '').lower()
        h = msg.height
        w = msg.width
        try:
            if 'yuy' in enc or 'yuv422' in enc or enc in ('yuyv', 'yuy2', 'yuv422p'):
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                # bytes per pixel (for packed formats typically 2)
                bpp = int(msg.step / w) if w > 0 else 2
                if bpp == 2:
                    arr = arr.reshape((h, w, 2))
                else:
                    arr2d = arr.reshape((h, msg.step))
                    arr2d = arr2d[:, :w * 2]
                    arr = arr2d.reshape((h, w, 2))
                # convert packed YUYV to BGR then to gray
                bgr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUY2)
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                return gray
            elif 'rgb' in enc or 'bgr' in enc:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                if 'rgb' in enc:
                    gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
                else:
                    gray = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
                return gray
            elif 'mono' in enc or 'gray' in enc:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, msg.step))[:, :w]
                # ensure shape (h,w)
                arr = arr.reshape((h, w))
                return arr.copy()
            else:
                # try best-effort: assume interleaved 3-channel
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                if arr.size == h * w * 3:
                    arr = arr.reshape((h, w, 3))
                    gray = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
                    return gray
                else:
                    self.get_logger().error(f'Unsupported/unknown encoding: {msg.encoding}')
                    return None
        except Exception as e:
            self.get_logger().error(f'Failed to convert message to gray: {e}')
            return None

    def image_callback(self, msg: Image):
        gray = self._msg_to_gray(msg)
        if gray is None:
            return

        # resize to 88x88
        try:
            gray_resized = cv2.resize(gray, (88, 88), interpolation=cv2.INTER_AREA)
        except Exception as e:
            self.get_logger().error(f'Resize error: {e}')
            return

        # write resized image to BRAM
        try:
            img_bytes = gray_resized.astype(np.uint8).tobytes()
            if not self._write_to_bram(img_bytes):
                self.get_logger().warn('Failed to write to BRAM')
        except Exception as e:
            self.get_logger().error(f'BRAM write prep error: {e}')

        # read image back from BRAM and publish
        bram_img = self._read_from_bram()
        if bram_img is not None:
            try:
                out = Image()
                out.header.stamp = self.get_clock().now().to_msg()
                out.header.frame_id = ''
                out.height = 88
                out.width = 88
                out.encoding = 'mono8'
                out.is_bigendian = 0
                out.step = 88
                out.data = bram_img.tobytes()
                self.pub.publish(out)
            except Exception as e:
                self.get_logger().error(f'Failed to publish BRAM image: {e}')

        # show locally for debug


def main(args=None):
    rclpy.init(args=args)
    node = ImageGraySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down')
        node.destroy_node()
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == '__main__':
    main()
