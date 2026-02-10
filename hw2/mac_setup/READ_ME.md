# Instructions to set up lpvDS learning on Mac 

Follow these instructions to set up proper compilation of the required librairies on Mac. You will then need to run 'setup_code_ch3' in Matlab.

1. First, download Xcode from the App Store. Then, open it once and accept the license

2. Once downloaded, open the package and follow the installation process.
 
3. Then, you have to open a terminal window at the folder meam6230-hw/hw2/mac_setup. 
Open a new terminal and cd to meam6230-hw/hw2/mac_setup
 

4. Then run the following command in this terminal:
bash Mac_config.sh.

5. Now in the matlab terminal, run 'mex -setup'

6. Then also in the matlab terminal, make sure you are in the hw2 folder (not hw2/mac_setup), run 'setup_code_ch3'

7. Again, in the matlab terminal, cd to 'meam6230-hw/libraries/book-thirdparty/yalmip/PENLABv104/source', and run 'mex mextrcolumn.c'

8. To test the setup, the file 'hw2/part2/ch3_ex4_lpvDS.m' should be run once (with 'est_options.type = 0'; it is like this by default, so you can just run).

If you encounter any issue with this installation, please contact tianyuli@seas.upenn.edu
