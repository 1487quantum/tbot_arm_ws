if [ ! -d "sph" ]; then
	mkdir sph
fi
cd sph
max=9
for i in `seq 0 $max`
do
	echo "Creating Ball_$i"
	if [ ! -d "ball${i}" ]; then
		mkdir ball$i
	fi
	cd ball$i
	echo "Copying template..."
	cp ../../ballN/* .
	echo "Updating model.config"
	sed -i "s/ballN/ball${i}/g" model.config	#replace more than one occurence you can add a g to the s-command
	sed -i "s/numberN/${i}/g" model.config
	echo "Updating sdf files"
	sed -i "s/ballN/ball${i}/g" model-1_4.sdf
	sed -i "s/ballN/ball${i}/g" model.sdf
	echo "Creating Material folder"
	if [ ! -d "materials" ]; then
		mkdir materials
	fi	
	cd materials
	if [ ! -d "scripts" ]; then
		mkdir scripts
	fi	
	cd scripts
	cp ../../../../ballN/materials/scripts/ballN.material ball$i.material
	sed -i "s/ballN/ball${i}/g" ball$i.material
	cd .. 
	if [ ! -d "textures" ]; then
		mkdir textures
	fi
	cd textures
	cp ../../../../ballN/materials/textures/ball$i.png .
	cd ../../../
	echo "===================================="
done
echo "Done!"