from subprocess import call, PIPE, Popen
from glob import glob
import re
from time import sleep
from os import getcwd, chdir, sep, path
import pyscreenshot as snipper

base_dir = getcwd()

#path_to_gc = "./build-pcl_custom_global_classification-Desktop-Default/global_classification"
gc_dir = "build-pcl_custom_global_classification-Desktop-Default/"
gc_exe = "./global_classification"

gc_scene_path = "-scene_path"
gc_args = ["-models_dir", "models/", "-training_dir", "training/", "-descriptor_name", "cvfh", "-nn:", "10"]
path_to_obj_seg = "build-obj_hand_seg-Desktop-Default/hand_obj_seg"
obj_seg_dir = "build-obj_hand_seg-Desktop-Default/"
obj_seg_exe = "./hand_obj_seg"

snapshot_dir = sep.join([base_dir, "screenshots"])
test_pcd_paths = glob("test_set/*")
gc_dir_abs = sep.join([getcwd(), gc_dir])
obj_seg_dir_abs = sep.join([base_dir, obj_seg_dir])

print test_pcd_paths

for pcd in test_pcd_paths:
    chdir(gc_dir_abs)
    pcd_abs = sep.join([base_dir, pcd])
    gc_call_str = [gc_exe,
                   gc_scene_path,
                   pcd_abs,
                   gc_args[0],
                   gc_args[1],
                   gc_args[2],
                   gc_args[3],
                   gc_args[4],
                   gc_args[5],
                   gc_args[6],
                   gc_args[7]]
    # print ' '.join(gc_call_str)
    p = Popen(' '.join(gc_call_str),
               shell=True,
               stdin=PIPE,
               stdout=PIPE,
               stderr=PIPE
              )
    sleep(8)
    print "please close pcl viewer window to continue"
    snipped = snipper.grab(bbox=(50, 20, 700, 450))     # X1,Y1,X2,Y2)
    #snipped.show()
    pcd_base = path.basename(pcd)
    pcd_name, extension = pcd_base.split('.')
    snipped.save(sep.join([snapshot_dir, pcd_name + "_" + "global_classification" + '.png']))

    output = p.stdout.read()
    # print output
    # print p.stderr.read()
    print gc_exe + " " + pcd
    classification_time = "FAILED"
    segementation_time = "FAILED"
    objects_segmented = "FAILED"

    m = re.findall('(?<=Objects segmented: )\d+', output)
    if m:
        objects_segmented = m[0]
    print "Objects segmented: " + objects_segmented

    m = re.findall('(?<=Segmentation done, )\d+', output)
    if m:
        segementation_time = m[0]
    print "Segmentation Time: " + segementation_time

    m = re.findall('(?<=Classification done, )\d+', output)
    if m:
        classification_time = m[0]
    print "Classification Time: " + classification_time

    #p.kill()
    p.wait()

    chdir(base_dir)
    chdir(obj_seg_dir_abs)

    obj_seg_call_str = [obj_seg_exe, pcd_abs]
    p = Popen(' '.join(obj_seg_call_str),
              shell=True,
              stdin=PIPE,
              stdout=PIPE,
              stderr=PIPE
              )

    sleep(8)
    print "done sleeping"
    snipped = snipper.grab(bbox=(10, 10, 700, 400))     # X1,Y1,X2,Y2)
    #snipped.show()
    pcd_base = path.basename(pcd)
    pcd_name, extension = pcd_base.split('.')
    snipped.save(sep.join([snapshot_dir, pcd_name + "_" + "obj_seg" + '.png']))

    output = p.stdout.read()

    print obj_seg_exe + " " + pcd
    classification_time = "FAILED"
    segementation_time = "FAILED"
    objects_segmented = "FAILED"

    m = re.findall('(?<=Objects segmented: )\d+', output)
    if m:
        objects_segmented = m[0]
    print "Objects segmented: " + objects_segmented

    m = re.findall('(?<=Segmentation done, )\d+', output)
    if m:
        segementation_time = m[0]
    print "Segmentation Time: " + segementation_time
    print "please close pcl viewer window to continue"


    p.wait()

    chdir(base_dir)





