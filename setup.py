import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

with open("requirements.txt", "r") as freq:
    required = freq.read().split()

setuptools.setup(
    name="ravestate",
    version="0.1.post3",
    url="https://github.com/roboy/ravestate",
    author="Roboy",
    author_email="info@roboy.org",

    description="Ravestate is a reactive library for real-time natural language dialog systems.",
    long_description=long_description,
    long_description_content_type="text/markdown",

    package_dir={'': 'modules'},
    packages=setuptools.find_packages("modules"),
    include_package_data=True,
    scripts=["rasta"],

    install_requires=required,
    python_requires='>=3.6',

    # TODO: Add classifiers
    # classifiers=[
    #     "Programming Language :: Python :: 3",
    #     "License :: OSI Approved :: MIT License",
    #     "Operating System :: OS Independent",
    # ],
)
