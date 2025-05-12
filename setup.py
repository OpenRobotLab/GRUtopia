import setuptools

with open('README.md', 'r') as fh:
    long_description = fh.read()

with open('requirements/runtime.txt', 'r') as fr:
    requirements = fr.readlines()

setuptools.setup(
    name='grutopia',
    version='2.1.0',
    author='APX103',
    author_email='lijialun@pjlab.org.cn',
    description='Easy to use omniverse isaac sim standalone package',
    long_description=long_description,
    long_description_content_type='text/markdown',
    packages=setuptools.find_packages(),
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
    install_requires=[i.strip() for i in requirements],
    python_requires='>=3.10',
)
