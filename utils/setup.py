from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['scripts/render_poses.py', 'scripts/render_poses_test.py'],
    packages=['utils'],
    package_dir={'': 'src'}
)