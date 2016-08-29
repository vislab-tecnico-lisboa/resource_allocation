
#!/bin/bash

##### Functions
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
	unzip piotr_toolbox.zip -d ../lib/
	echo "Downloading MOT toolbox"
	wget https://motchallenge.net/data/devkit.zip
	unzip devkit.zip -d ../lib/mot
	echo "Done"
}


sudo apt-get install unzip
mkdir lib/mot
mkdir data/res
mkdir downloads
cd downloads
rm -rf *

get_toolboxs
cp ../data/all.txt ../lib/mot/seqmaps/
get_datasets


cd ..
