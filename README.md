# MEAM6230 Homework (Spring 2025)

Maintainers: Contact 2025 Spring semester TAs

This repo is the course homework repo for MEAM6230 @ Penn.

---

## Setting up MATLAB

Visit [MATLAB for SEAS Students](https://cets.seas.upenn.edu/software/matlab/student/) to obtain the activation code and relevant information.

#### Register for an account:

- Visit https://www.mathworks.com/.

- Click on the Sign In link.

- Important: Enter your email address as xxx@upenn.edu (not xxx@seas.upenn.edu).

- Log in with your PennKey at the familiar Penn WebLogin page.

- You will be prompted again for an email address:

    - If you already have a MathWorks account, enter the email address associated with that account.

    - If you do not already have a MathWorks account, enter xxx@upenn.edu.


#### To download and activate Matlab:

- While logged into your new MathWorks account, click your username in the top right corner and then MyAccount in the pull down menu. Or as soon as you sign in, there will be a download button as well

- Click the download arrow to download the installer and follow the instructions.

- When asked for installation method, select Log in with a MathWorks account and select Next.

- Please remember to use the appropriate email address (xxxx@upenn.edu for new accounts).

- When prompted to select a license or Activation Key, click Select a license and then click 353265 Student - Standalone Named User for our student license. Then click Next.

- When prompted to choose an installation folder, either browse to a custom folder path or stick with the default and then click Next.

- Continue to click Next to install the products.


#### Installation

Follow the installation program. You will need a version of Matlab from 2019 or higher with the following toolboxes :
- Control System
- Curve Fitting
- Deep Learning
- Image Processing 
- Model Predictive Control
- Optimization
- Robotic System
- ROS 
- Signal Processing
- Statistics and Machine Learning


## Homework Structure:

In general, the entire repo will be in the format of
```
This repo
    |
    |--hw1
    |    |--lib
    |    |--test
    |    |--{some main files}
    |...
    |--hwN
    |     |--lib
    |     |--test
    |     |--{some main files}
    |--libraries
    |--README.md
```

Your goal is to implement all the functions in the lib folder. The main files are for you to understand the context of the functions and create visualizations. The ```test``` folder is what we use to give you grade. It serves as an autograder for you to know if you pass all the tests. It is recommended to first read the main files to understand the context and then start working on the functions.

## Usage

#### Running main files
In case you are not familiar with MATLAB. The most straighforward way to run any program (for example the main files) is to click on the green arrow button (```says "Run"```) on top in the ```Editor``` tab.

#### Running test files
The test files are called ```pcode```, which the source code are hidden. We use this as an autograder for you to test. To use the test file, you will cd into the test folder and call the ```.p``` file name in the MATLAB command window
```
cd test
{test file name without the .p}
```