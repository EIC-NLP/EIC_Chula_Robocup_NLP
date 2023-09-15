from setuptools import setup, find_packages

from nlp_client import __version__


setup(
    name='nlp_client',
    version=__version__,
    description='A nlp_client to use with EIC 2023 Robocup NLP server.',

    url='https://github.com/GameTL',
    author='Tinapat Limsila',
    author_email='limsila.limsila@gmail.com',



    classifiers=[
        'Intended Audience :: Developers',

        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
    ],
)
