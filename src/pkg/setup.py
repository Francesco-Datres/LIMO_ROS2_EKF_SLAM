from setuptools import find_packages, setup

package_name = 'mio_primo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='francesco',
    maintainer_email='francesco@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'esegui_prova = mio_primo_pkg.nodo_prova:main',
        'avvia_simulazione = mio_primo_pkg.simulatore:main',
        'avvia_nodo = mio_primo_pkg.simulatore_real:main',
        ],
    },
)
