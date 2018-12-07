import setuptools

setuptools.setup(
    name="reggol",
    version="0.1.0",
    url="https://github.com/roboy/ravestate",
    author="Roboy",
    author_email="info@roboy.org",

    description="Reggol is a convenience layer over python's logging module.",
    long_description="""
### Reggol is your weird best friend when it comes to logging.

## Usage

It takes away some arguments frome your command line args; namely
`--loglevel` and `--logpath`.

You may use `reggol.help_string()` to append reggols
argument help to your own command line.

Use `reggol.get_logger()` in any of your code files for some powerful,
beautiful synced console-file logging. Woah!

## What else to say

### Happy logging!
""",
    long_description_content_type="text/markdown",

    package_dir={'': 'modules'},
    packages=setuptools.find_packages("modules", exclude=["ravestate*"]),
    include_package_data=False,

    install_requires=[],
    python_requires='>=3.6',

    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)
