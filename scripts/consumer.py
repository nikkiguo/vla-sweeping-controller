import mmap
import posix_ipc
import numpy as np
import time
import cv2

# Matching the C++ struct layout for shared memory
dtype = np.dtype([
    ('frame_index', 'u8'),
    ('joint_states', 'f8', (6,)),
    ('camera_pixels', 'u1', (640 * 480 * 3,))
])

def run_consumer():
    # Create or open the shared memory segment
    shm = posix_ipc.SharedMemory("/robot_data_shm")

    # Map the shared memory into address space
    buf = mmap.mmap(shm.fd, shm.size)
    data = np.frombuffer(buf, dtype=dtype, count=1)

    print("Connected to SHM. Reading data...")
    cv2.namedWindow("VLA Sweeping Sandbox visual")

    try:
        while True:
            # Access the data directly from the buffer
            current_frame = data[0]['frame_index']
            joints = data[0]['joint_states']
            camera_pixels = data[0]['camera_pixels']

            # Display and print state every 10 frames
            if current_frame % 10 == 0:
                image = camera_pixels.reshape((480, 640, 3))
                image = image[::-1, :, :]  # Flip vertically
                cv2.imshow("VLA Sweeping Sandbox visual", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                if cv2.waitKey(30) & 0xFF == ord('q'):
                    break
                print(f"Frame: {current_frame} | Joint[0]: {joints[0]:.4f} | Joint[1]: {joints[1]:.4f} | Joint[2]: {joints[2]:.4f} | Joint[3]: {joints[3]:.4f} | Joint[4]: {joints[4]:.4f} | Joint[5]: {joints[5]:.4f}")
    except KeyboardInterrupt:
        print("Stopping consumer...")
    finally:
        buf.close()

if __name__ == "__main__":
    run_consumer()