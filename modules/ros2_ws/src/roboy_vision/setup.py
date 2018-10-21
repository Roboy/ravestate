from setuptools import setup, find_packages
import os

DIR = os.path.dirname(os.path.realpath(__file__))
package_name = 'roboy_vision'


with open(os.path.join(DIR, 'README.rst')) as readme_file:
    readme = readme_file.read()

with open(os.path.join(DIR, 'HISTORY.rst')) as history_file:
    history = history_file.read()

requirements = [
    'face_recognition_models>=0.3.0',
    'Click>=6.0',
    'dlib>=19.7',
    'numpy',
    'Pillow'
]

test_requirements = [
    'tox',
    'flake8==2.6.0'
]

setup(
    name=package_name,
    version='1.2.3',
    packages=find_packages(),
    description="Roboy complete vision pack",
    long_description=readme + '\n\n' + history,
    author="Emilia Lozinska",
    author_email='em.lozinska@gmail.com',
    url='https://github.com/ageitgey/face_recognition',
    package_data={
        'face_recognition': ['models/*.dat']
    },
    entry_points={
        'console_scripts': [
            'face_recognition=face_recognition.examples.facerec_ros_webcam:main',
            'webcam=webcam_publisher.webcam:main'
        ]
    },
    install_requires=requirements,
    license="BSD",
    zip_safe=False,
    keywords='face_recognition',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        "Programming Language :: Python :: 2",
        'Programming Language :: Python :: 2.6',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
    ],
    test_suite='tests',
    tests_require=test_requirements
)
