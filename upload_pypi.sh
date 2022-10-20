if tox; then
    rm -rf ./build
    rm -rf ./dist

    python3 setup.py bdist_wheel sdist

    twine upload dist/*
else
    echo command returned some error
fi