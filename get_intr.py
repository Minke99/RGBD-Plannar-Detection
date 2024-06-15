import pyrealsense2 as rs

pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

profile = pipeline.get_active_profile()

color_stream = profile.get_stream(rs.stream.color)

intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

print("Width:", intrinsics.width)
print("Height:", intrinsics.height)
print("PPX:", intrinsics.ppx)
print("PPY:", intrinsics.ppy)
print("FX:", intrinsics.fx)
print("FY:", intrinsics.fy)
print("Distortion Model:", intrinsics.model)
print("Coefficients:", intrinsics.coeffs)

pipeline.stop()
