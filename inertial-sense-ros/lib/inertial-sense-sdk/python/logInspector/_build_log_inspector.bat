@echo off

echo Building Log Inspector
echo.

cd ../

python -m pip install logInspector/

cd logInspector

python setup.py build_ext --inplace

REM timeout 5 /nobreak
