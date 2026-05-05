from setuptools import find_packages, setup
from setuptools.command.develop import develop as _develop
import os
import warnings


class DevelopWithUninstall(_develop):
    """
    Some colcon/ament-python workflows may call:
      setup.py develop --uninstall --editable ...
    But setuptools in some environments doesn't recognize --uninstall.
    We accept the option for compatibility (no-op), then run normal develop.
    """

    user_options = _develop.user_options + [
        ("uninstall", None, "Compatibility no-op"),
        ("editable", None, "Compatibility no-op"),
        ("build-directory=", None, "Compatibility no-op"),
        ("script-dir=", None, "Compatibility no-op (from setup.cfg)"),
    ]

    def initialize_options(self):
        # Suppress setuptools develop deprecation noise in colcon workflows.
        dep_warn = getattr(__import__("setuptools.command.develop", fromlist=["DevelopDeprecationWarning"]),
                           "DevelopDeprecationWarning", Warning)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", dep_warn)
            super().initialize_options()
        # colcon may pass it as a flag; ignore the value.
        self.uninstall = None
        self.editable = None
        self.build_directory = None
        self.script_dir = None

    def run(self):
        # Reduce pip noise in colcon editable installs.
        os.environ.setdefault("PIP_DISABLE_PIP_VERSION_CHECK", "1")
        old_virtual_env = os.environ.pop("VIRTUAL_ENV", None)
        try:
            super().run()
        finally:
            if old_virtual_env is not None:
                os.environ["VIRTUAL_ENV"] = old_virtual_env

package_name = 'traffic_editor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    cmdclass={
        "develop": DevelopWithUninstall,
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt5>=5.15.0', 'pyyaml>=5.4', 'qt-material>=2.12'],
    zip_safe=True,
    maintainer='Fleet Admin',
    maintainer_email='admin@fleet.com',
    description='Traffic map editor for fleet management',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'traffic_editor = traffic_editor.main_window:main',
        ],
    },
)
