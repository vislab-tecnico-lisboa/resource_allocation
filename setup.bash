
#!/bin/bash

##### Functions
function get_annotations
{
    echo "Downloading annotations"
    wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/annotations.zip
    unzip annotations.zip -d ../matlab/vbb/data-USA
    echo "Done"
}

function get_results
{
    echo "Downloading results"
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/res/ACF.zip
	unzip ACF -d ../matlab/vbb/data-USA/res
    echo "Done"
}

function get_labeling_code
{
    echo "Downloading labeling code"
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/code/code3.2.1.zip
	unzip code3.2.1.zip -d ../matlab/vbb
    echo "Done"
}

function get_datasets
{
	echo "Downloading datasets"
	wget https://motchallenge.net/data/MOT16.zip
	unzip MOT16.zip -d ../data
    	echo "Done"
}

function get_toolboxs
{	
	echo "Downloading piotr dollar toolbox"
	wget https://pdollar.github.io/toolbox/archive/piotr_toolbox.zip
	unzip piotr_toolbox.zip -d ../matlab/
	echo "Downloading MOT toolbox"
	wget https://motchallenge.net/data/devkit.zip
	unzip devkit.zip -d ../matlab/mot
	echo "Done"
}

sudo apt-get install unzip
mkdir matlab/mot
mkdir data/res
mkdir downloads
cd downloads
rm -rf *

get_toolboxs
get_datasets


cd ..
