import os
import read_write_dense

framenum = 1173
datasetdir = ".\\dataset0\\"

frames = [6, 274, 332, 399, 431, 484, 588, 590, 673, 694, 808, 870, 890, 891, 959, 1018, 1020, 1025, 1099, 1153]

for i in range(framenum):
	tempdir = datasetdir + "frame_" + str(i) + "\\"
	imagesdir = datasetdir + "frame_" + str(i) + "\\" + "images\\"
	sparsedir = datasetdir + "frame_" + str(i) + "\\" + "sparse\\"
	os.system("mkdir " + sparsedir)
	densedir = datasetdir + "frame_" + str(i) + "\\" + "dense\\"
	os.system("mkdir " + densedir)

	depthmapsdir = densedir + "stereo\\" + "depth_maps\\"
	outputdir = tempdir + "depths\\"

	os.system("colmap feature_extractor --database_path database.db --image_path {} --ImageReader.camera_model PINHOLE".format(imagesdir))
	os.system("colmap exhaustive_matcher --database_path database.db")
	os.system("colmap point_triangulator --database_path database.db --image_path {} --input_path created/sparse --output_path {}".format(imagesdir, sparsedir))
	os.system("colmap image_undistorter --image_path {} --input_path {} --output_path {}".format(imagesdir, sparsedir, densedir))
	os.system("colmap patch_match_stereo --workspace_path {} --workspace_format COLMAP --PatchMatchStereo.geom_consistency true".format(densedir))
	os.system("del database.db")

	for j in range(12):
		binjdir = depthmapsdir + str(j) + '.png.' + 'geometric' + '.bin'
		if os.path.exists(binjdir):
			read_write_dense.bin2depth(j, binjdir, outputdir) # photometric

	os.system("rd /s/q " + densedir)
	os.system("rd /s/q " + sparsedir)
	# os.system("del " + outputdir + "*.yuv")
