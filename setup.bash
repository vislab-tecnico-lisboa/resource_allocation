
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
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set00.tar
	tar xopf set00.tar
        mv set00 ../matlab/vbb/data-USA/videos
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set01.tar
	tar xopf set01.tar
        mv set01 ../matlab/vbb/data-USA/videos
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set02.tar
	tar xopf set02.tar
        mv set02 ../matlab/vbb/data-USA/videos
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set03.tar
	tar xopf set03.tar
        mv set03 ../matlab/vbb/data-USA/videos
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set04.tar
	tar xopf set04.tar
        mv set04 ../matlab/vbb/data-USA/videos
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set05.tar
	tar xopf set05.tar
        mv set05 ../matlab/vbb/data-USA/videos
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set06.tar
	tar xopf set06.tar
        mv set06 ../matlab/vbb/data-USA/videos
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set07.tar
	tar xopf set07.tar
        mv set07 ../matlab/vbb/data-USA/videos
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set08.tar
	tar xopf set08.tar
        mv set08 ../matlab/vbb/data-USA/videos
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set09.tar
	tar xopf set09.tar
        mv set09 ../matlab/vbb/data-USA/videos
	wget http://www.vision.caltech.edu/Image_Datasets/CaltechPedestrians/datasets/USA/set10.tar
	tar xopf set10.tar
        mv set10 ../matlab/vbb/data-USA/videos
    	echo "Done"
}

function get_toolbox
{	
	echo "Downloading piotr dollar toolbox"
	wget https://pdollar.github.io/toolbox/archive/piotr_toolbox.zip
	unzip piotr_toolbox.zip -d ../matlab/
	echo "Done"
}

sudo apt-get install unzip
mkdir downloads
cd downloads
rm -rf *

get_toolbox



cd ..
