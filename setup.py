import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

required_url = []
required = []
with open("requirements.txt", "r") as freq:
    for line in freq.read().split():
        if "://" in line:
            required_url.append(line)
        else:
            required.append(line)

packages = setuptools.find_packages("modules", exclude=["reggol*"])

setuptools.setup(
    name="ravestate",
    version="0.3.0",
    url="https://github.com/roboy/ravestate",
    author="Roboy",
    author_email="info@roboy.org",

    description="Ravestate is a reactive library for real-time natural language dialog systems.",
    long_description=long_description,
    long_description_content_type="text/markdown",

    package_dir={package: 'modules/'+package for package in packages},
    packages=packages,
    include_package_data=True,
    package_data={
        'ravestate_phrases_basic_en': ['en/*.yml'],
        'ravestate_ontology': ['ravestate_ontology.yml'],
        'ravestate_roboyqa': ['answering_phrases/RoboyInfoList.yml']
    },
    scripts=["rasta"],

    install_requires=required + ["reggol"],
    dependency_links=required_url,
    python_requires='>=3.7',

    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)
