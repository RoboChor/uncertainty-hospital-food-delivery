import os
import argparse
import docker

def filter_container(img_base,c_id, c_name):
    client = docker.from_env()
    all_ctns = client.containers.list()
    sel_ctns = []
    for c in all_ctns:    
        # print(im.tags[0])
        # print(img_base in im.tags[0])
        # print(f"The image name is {c.image.tags[0]}")
        if img_base in c.image.tags[0]:
            cn = {
                "id": c.short_id,
                "date": c.attrs['Created'],        
                "status": c.status,
                "name": c.name,
                "image": c.image.tags[0],
                "image_id": c.image.id[7:15],     
            }
            sel_ctns.append(cn)
    # Filter the containers to obtain the one
    # If there are no containers active
    if len(sel_ctns) == 0:
        return None 
       
    # Filter the containers to obtain the one
    f_id, f_name = c_id, c_name
    if f_id != '' or f_name != '':
        fsc = [c for c in sel_ctns if (f_id!='' and c['id'].startswith(f_id)) or (f_name!='' and c['name']==f_name)]
        if len(fsc) > 0:
            sc = fsc[0]
        else:
            return None
    else:
        sc = sel_ctns[0]
    # print(f"Result selected: {sc}")
    return sc

if __name__ == "__main__":
    img_base = "hfd:v1.1"
    cnt_name = "hfd_container"
    sc = filter_container(img_base,"",cnt_name)
    if sc is None:
        print("ERR1: No container found with such id or name.")
        exit(1)
    sc_id = sc['id']
    cmd = f'gnome-terminal -- bash -c "docker exec -it {sc_id} bash; exec bash"'
    os.popen(cmd)