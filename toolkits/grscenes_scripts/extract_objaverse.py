import argparse
import os

from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

'''
This script is to extract materials from usd objects transformed from objaverse dataset.
The usd hierarchy is:
World
----Looks
--------<material_1>
--------<material_2>
----Sketchfab_model
--------<model_component_1>
--------<model_component_2>
'''
transforms_name_set = set(['xformOp:transform', 'xformOp:translate', 'xformOp:orient', 'xformOp:scale', 'xformOpOrder'])


def is_absolute(path):
    if path[0] == '/':
        return True
    return False


def simplify_path(path):
    # print(path)
    elements = path.split('/')
    simp = []
    for e in elements:
        if e == '':
            # continue
            simp.append(e)
        elif e == '.':
            if len(simp) == 0:
                simp.append(e)
            continue
        elif e == '..':
            if len(simp) != 0 and simp[-1] != '..' and simp[-1] != '.':
                simp = simp[:-1]
            else:
                simp.append(e)
        else:
            simp.append(e)
    new_path = '/'.join(simp)
    return new_path


def remove_parent_prefix(path):
    elements = path.split('/')
    for i, e in enumerate(elements):
        if e == '..':
            continue
        else:
            elements = elements[i:]
            new_path = os.path.join(*elements)
            return new_path


def copyfile(src, dest):
    srct = simplify_path(src)
    destt = simplify_path(dest)
    if not os.path.exists(srct):
        print(srct, 'does not exist!', src)
        pass
    else:
        # f_name = destt.split('/')[-1]
        _d_list = destt.split('/')[:-1]
        print(f'_d_list is {_d_list}')
        if not os.path.exists('/'.join(_d_list)):
            os.makedirs(os.path.join('/'.join(_d_list)))
        # if destt[-3:] in ['png', 'jpg', 'jpeg'] and not f_name in file_name_set:
        #     file_name_set.add(f_name)
        #     print('from', srct, 'to', destt)

        os.system(f'cp {srct} {destt}')
    return True


def recursive_copy(  # noqa: C901
    prim_a, prim_b, copy_transform=True, src_prepend='.', dest_prepend='.', ref_prepend='.'
):
    '''
    There are three kinds of properties could appear in a prim:
    1. attribute: some attributes
    2. relationship: internal prim path
    3. reference: external usd file path
    '''
    looks_b_path = '/Root/Looks'
    others_b_path = '/Root/Others'
    stage_a = prim_a.GetStage()
    stage_b = prim_b.GetStage()
    print(f'==========prim_a {prim_a.GetPath()}, prim_b {prim_b.GetPath()}==============')
    # print(prim_a.GetName(), prim_a.GetReferences(), prim_a.HasAuthoredReferences())

    # process relationships
    rels_a = prim_a.GetRelationships()
    for rel in rels_a:
        # print(rel)
        # targets = []
        # print(rel.GetName())
        # rel.GetTargets(targets)
        # print(targets)
        # print(rel.GetName(), rel.GetTargets())
        rel_name = rel.GetName()
        targets = rel.GetTargets()

        if len(targets) > 0:
            rel_b = prim_b.CreateRelationship(rel_name, custom=False)

        # print(rel_name, targets, prim_a.GetPath())
        if rel_name == 'material:binding':
            # print(rel.GetAttributes())
            # print(prim_a.GetProperties())
            # help(rel)
            # print(rel, rel.HasAuthoredTargets())
            for t in targets:
                material_a = stage_a.GetPrimAtPath(t)
                material_name = material_a.GetName()
                material_b_path = os.path.join(looks_b_path, material_name)
                material_b = stage_b.GetPrimAtPath(material_b_path)
                # print(Usd.Object.IsValid(p))
                if not Usd.Object.IsValid(material_b):
                    material_b = stage_b.DefinePrim(material_b_path, material_a.GetTypeName())
                    recursive_copy(material_a, material_b, src_prepend=src_prepend, dest_prepend=dest_prepend)

                rel_b.AddTarget(material_b_path)
                strength = UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(rel)
                # print(strength)
                UsdShade.MaterialBindingAPI.SetMaterialBindingStrength(rel_b, strength)
                # print(material_b.GetTypeName())
                # binding = UsdShade.MaterialBindingAPI.Apply(prim_b)
                # binding.Bind(material_b, UsdShade.Tokens.strongerThanDescendants, UsdShade.Tokens.allPurpose)

                pass
        elif rel_name == 'physics:body0' or rel_name == 'physics:body1':
            for t in targets:
                rel_p_a = stage_a.GetPrimAtPath(t)
                # rel_p_a_name = rel_p_a.GetName()
                # rel_p_b_path = os.path.join(others_b_path, rel_p_a_name)
                # rel_p_b = stage_b.GetPrimAtPath(rel_p_b_path)
                # # print(Usd.Object.IsValid(p))
                # if not Usd.Object.IsValid(rel_p_b):
                #     rel_p_b = stage_b.DefinePrim(rel_p_b_path, rel_p_a.GetTypeName())
                #     recursive_copy(rel_p_a, rel_p_b)
                if not Usd.Object.IsValid(rel_p_a):
                    print('invalid target', t)
                    continue
                rel_p_a_name = rel_p_a.GetName()
                rel_p_b_path = os.path.join('/Root', 'Instance', rel_p_a_name)
                rel_b.AddTarget(rel_p_b_path)
        else:
            for t in targets:
                rel_p_a = stage_a.GetPrimAtPath(t)
                rel_p_a_name = rel_p_a.GetName()
                rel_p_b_path = os.path.join(others_b_path, rel_p_a_name)
                rel_p_b = stage_b.GetPrimAtPath(rel_p_b_path)
                if not Usd.Object.IsValid(rel_p_b):
                    rel_p_b = stage_b.DefinePrim(rel_p_b_path, rel_p_a.GetTypeName())
                    recursive_copy(rel_p_a, rel_p_b, src_prepend=src_prepend, dest_prepend=dest_prepend)

                rel_b.AddTarget(rel_p_b_path)

    ref_prepend = '.'
    prim = prim_a
    while Usd.Object.IsValid(prim):
        if prim.HasAuthoredReferences():
            tmp_references_list = []
            for prim_spec in prim.GetPrimStack():
                tmp_references_list.extend(prim_spec.referenceList.prependedItems)
            # print('tmp_references_list',tmp_references_list, prim_a.GetPath(), prim.GetPath())
            tmp_ref_prepend_path = os.path.join(*(str(tmp_references_list[0].assetPath).split('/')[:-1]))
            ref_prepend = simplify_path(os.path.join(tmp_ref_prepend_path, ref_prepend))

        prim = prim.GetParent()

    # print(prim_a.GetName(), "copy_transform:", copy_transform)
    if copy_transform:
        pass

    # process attributes
    attributes = prim_a.GetAttributes()
    skip_attrs = set()
    for attr in attributes:
        name = attr.GetName()
        typename = attr.GetTypeName()
        val = attr.Get()
        # if name in transforms_name_set:
        #     print(name, typename, val)

        if name in skip_attrs:
            continue

        # var = attr.GetVariability()
        connections = attr.GetConnections()

        # if prim_a.GetTypeName() in ['Material', 'Shader']:
        #     print(prim_a.GetName(), name, typename, val, attr.GetConnections(), attr.GetColorSpace(), type(attr.GetColorSpace()))

        # if not copy_transform:
        #     if name in transforms_name_set:
        #         continue
        if name in transforms_name_set:
            continue

        # if val is not None or len(connections) > 0:
        new_attr = prim_b.CreateAttribute(name, typename, custom=False)

        # new_attr.SetVariability(var)
        if prim_a.IsA(UsdPhysics.PrismaticJoint) or prim_a.IsA(UsdPhysics.RevoluteJoint):
            if name == 'physics:jointEnabled':
                val = 0

        if val is not None:
            new_attr.Set(val)
            input_attr_name = f'inputs:{name}'
            if prim_a.HasAttribute(input_attr_name):
                skip_attrs.add(input_attr_name)
                if prim_b.HasAttribute(input_attr_name):
                    new_input_attr = prim_b.GetAttribute(input_attr_name)
                else:
                    new_input_attr = prim_b.CreateAttribute(input_attr_name, typename, custom=False)

                new_input_attr.Set(val)

        if len(connections) > 0:
            for c in connections:
                connection_b = str(c)
                new_attr.AddConnection(connection_b)

        if typename == 'asset':
            colorspace = attr.GetColorSpace()
            if len(colorspace) > 0:
                new_attr.SetColorSpace(colorspace)

            if val is None:
                continue
            # print(val.GetTypeName())

            # filepath = str(val)[1:-1]
            filepath = val.path
            # print(filepath)
            # ap = Sdf.AssetPath(filepath)
            # print(ap.path)
            if is_absolute(filepath) or str(filepath).startswith('http://'):
                new_attr.Set(Sdf.AssetPath(filepath))
            else:

                # updated_filepath = simplify_path(os.path.join(ref_prepend, filepath))
                updated_filepath = remove_parent_prefix(filepath)
                # updated_filepath = filepath
                # print(prim_a.GetPath(), attr.GetName(), ref_prepend, filepath, updated_filepath, src_prepend)
                new_attr.Set(Sdf.AssetPath(updated_filepath))

                # print(prim_a.GetName(), name, src_prepend, filepath)
                source_file_path = os.path.join(src_prepend, ref_prepend, filepath)
                # dest_file_path = os.path.join(dest_prepend, ref_prepend, filepath)
                dest_file_path = os.path.join(dest_prepend, updated_filepath)
                # print(dest_prepend,'=====================', ref_prepend)
                copyfile(source_file_path, dest_file_path)

                # print('========================')
                # print('source_file_path', source_file_path)
                # print('dest_file_path', dest_file_path)
                # print('filepath', filepath)
                # print('updated_filepath', updated_filepath)
                #     import ipdb
                #     ipdb.set_trace()
                # print(dest_prepend)

    # process geometry primvars
    if prim_a.IsA(UsdGeom.Mesh):
        api_a = UsdGeom.PrimvarsAPI(prim_a)
        api_b = UsdGeom.PrimvarsAPI(prim_b)
        primvars = api_a.GetPrimvars()
        for var in primvars:
            # print(var.GetName(), var.GetInterpolation())
            name = var.GetName()
            it = var.GetInterpolation()
            if it != 'constant':
                var_b = api_b.GetPrimvar(name)
                var_b.SetInterpolation(it)
        # print(primvars)

    # process children
    children = prim_a.GetChildren()
    prim_b_path = str(prim_b.GetPath())
    for child in children:
        typename = child.GetTypeName()
        name = child.GetName()
        # print(name)
        child_path = os.path.join(prim_b_path, name)
        new_child = stage_b.DefinePrim(child_path, typename)
        recursive_copy(child, new_child, src_prepend=src_prepend, dest_prepend=dest_prepend)

    pass


# create new stage for extracted materials
def extract_mtl(usd_path, mtl_path, usd_file):
    src_path = os.path.join(usd_path, usd_file)
    dst_path = os.path.join(mtl_path, usd_file)
    usd_stage = Usd.Stage.Open(src_path)
    mtl_stage = Usd.Stage.CreateNew(dst_path)
    mtl_stage.GetRootLayer().customLayerData = usd_stage.GetRootLayer().customLayerData

    world_prim = usd_stage.GetDefaultPrim()
    root_new = mtl_stage.DefinePrim(world_prim.GetPath(), world_prim.GetTypeName())
    mtl_stage.SetDefaultPrim(root_new)
    looks = world_prim.GetPrimAtPath('Looks')
    materials = looks.GetChildren()
    mtl_stage.DefinePrim(looks.GetPath(), looks.GetTypeName())
    for mtl in materials:
        new_mtl = mtl_stage.DefinePrim(mtl.GetPath(), mtl.GetTypeName())
        recursive_copy(mtl, new_mtl, copy_transform=False, src_prepend=usd_path, dest_prepend=mtl_path)

    mtl_stage.GetRootLayer().Save()


def main():
    parser = argparse.ArgumentParser('Extract Materials from usd')
    parser.add_argument(
        '--usd_path',
        type=str,
        default=None,
        help='The path of the usd assets',
    )

    parser.add_argument(
        '--material_path',
        type=str,
        default=None,
        help='The path of the output material assets',
    )

    args, unknown_args = parser.parse_known_args()
    if not os.path.exists(args.material_path):
        os.makedirs(args.material_path)

    usd_files = [_ for _ in os.listdir(args.usd_path) if _[-3:] == 'usd']
    for usd_file in usd_files:
        print(f'usd_file is {usd_file}')
        extract_mtl(args.usd_path, args.material_path, usd_file)


if __name__ == '__main__':
    main()
