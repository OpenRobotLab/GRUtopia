# Copyright (C) 2024, Princeton University.
# This source code is licensed under the BSD 3-Clause license found in the LICENSE file in the root directory of this source tree.

# Authors:
# - Karhan Kayan

import os
import random

import bpy
from infinigen.assets.static_assets.base import StaticAssetFactory
from infinigen.core.tagging import tag_support_surfaces
from infinigen.core.util.math import FixedSeed


def static_category_factory(
    path_to_assets: str,
    tag_support=False,
    x_dim: float = None,
    y_dim: float = None,
    z_dim: float = None,
    rotation_euler: tuple[float] = None,
) -> StaticAssetFactory:
    """
    Create a factory for external asset import.
    tag_support: tag the planes of the object that are parallel to xy plane as support surfaces (e.g. shelves)
    x_dim, y_dim, z_dim: specify ONLY ONE dimension for the imported object. The object will be scaled accordingly.
    rotation_euler: sets the rotation of the object in euler angles. The object will not be rotated if not specified.
    """

    class StaticCategoryFactory(StaticAssetFactory):
        def __init__(self, factory_seed, coarse=False):
            super().__init__(factory_seed, coarse)
            with FixedSeed(factory_seed):
                self.path_to_assets = path_to_assets
                self.tag_support = tag_support
                self.asset_dir = path_to_assets
                self.x_dim, self.y_dim, self.z_dim = x_dim, y_dim, z_dim
                self.rotation_euler = rotation_euler
                asset_files = [
                    f for f in os.listdir(self.asset_dir) if f.lower().endswith(tuple(self.import_map.keys()))
                ]
                if not asset_files or len(asset_files) == 0:
                    raise ValueError(f'No valid asset files found in {self.asset_dir}')
                self.asset_file = random.choice(asset_files)

        def create_asset(self, **params) -> bpy.types.Object:
            file_path = os.path.join(self.asset_dir, self.asset_file)
            imported_obj = self.import_file(file_path)
            if self.x_dim is not None or self.y_dim is not None or self.z_dim is not None:
                # check only one dimension is provided
                if sum([1 for dim in [self.x_dim, self.y_dim, self.z_dim] if dim is not None]) != 1:
                    raise ValueError('Only one dimension can be provided')
                if self.x_dim is not None:
                    scale = self.x_dim / imported_obj.dimensions[0]
                elif self.y_dim is not None:
                    scale = self.y_dim / imported_obj.dimensions[1]
                else:
                    scale = self.z_dim / imported_obj.dimensions[2]
                imported_obj.scale = (scale, scale, scale)
            if self.tag_support:
                tag_support_surfaces(imported_obj)

            if imported_obj:
                return imported_obj
            else:
                raise ValueError(f'Failed to import asset: {self.asset_file}')

    return StaticCategoryFactory


# Create factory instances for different categories
StaticBookFactory = static_category_factory('infinigen/assets/static_assets/source/Book')
StaticLaptopFactory = static_category_factory('infinigen/assets/static_assets/source/Laptop')
StaticSinkFactory = static_category_factory('infinigen/assets/static_assets/source/Sink')
StaticShoeFactory = static_category_factory('infinigen/assets/static_assets/source/Shoe')
StaticPlateFactory = static_category_factory('infinigen/assets/static_assets/source/Plate')
StaticBicycleFactory = static_category_factory('infinigen/assets/static_assets/source/Bicycle')
StaticFanFactory = static_category_factory('infinigen/assets/static_assets/source/Fan')
StaticWashingmachineFactory = static_category_factory('infinigen/assets/static_assets/source/Washingmachine')
StaticShoecabinetFactory = static_category_factory('infinigen/assets/static_assets/source/Shoecabinet', z_dim=2.5)
StaticOtherFactory = static_category_factory('infinigen/assets/static_assets/source/Other')
StaticDeskFactory = static_category_factory('infinigen/assets/static_assets/source/Desk')
StaticTrayFactory = static_category_factory('infinigen/assets/static_assets/source/Tray')
StaticToyFactory = static_category_factory('infinigen/assets/static_assets/source/Toy')
StaticDoorFactory = static_category_factory('infinigen/assets/static_assets/source/Door')
StaticBedFactory = static_category_factory('infinigen/assets/static_assets/source/Bed')
StaticElectriccookerFactory = static_category_factory('infinigen/assets/static_assets/source/Electriccooker')
StaticClockFactory = static_category_factory('infinigen/assets/static_assets/source/Clock')
StaticPlantFactory = static_category_factory('infinigen/assets/static_assets/source/Plant')
StaticTvstandFactory = static_category_factory('infinigen/assets/static_assets/source/Tvstand')
StaticRefrigeratorFactory = static_category_factory('infinigen/assets/static_assets/source/Refrigerator')
StaticFaucetFactory = static_category_factory('infinigen/assets/static_assets/source/Faucet')
StaticStoolFactory = static_category_factory('infinigen/assets/static_assets/source/Stool')
StaticBathtubFactory = static_category_factory('infinigen/assets/static_assets/source/Bathtub')
StaticCabinetFactory = static_category_factory('infinigen/assets/static_assets/source/Cabinet', z_dim=2.5)
StaticCouchFactory = static_category_factory('infinigen/assets/static_assets/source/Couch')
StaticLightFactory = static_category_factory('infinigen/assets/static_assets/source/Light')
StaticClothesFactory = static_category_factory('infinigen/assets/static_assets/source/Clothes')
StaticBowlFactory = static_category_factory('infinigen/assets/static_assets/source/Bowl')
StaticPictureFactory = static_category_factory('infinigen/assets/static_assets/source/Picture')
StaticBookshelfFactory = static_category_factory('infinigen/assets/static_assets/source/Bookshelf')
StaticMicrowaveFactory = static_category_factory('infinigen/assets/static_assets/source/Microwave')
StaticToiletFactory = static_category_factory('infinigen/assets/static_assets/source/Toilet')
StaticBlanketFactory = static_category_factory('infinigen/assets/static_assets/source/Blanket')
StaticCupFactory = static_category_factory('infinigen/assets/static_assets/source/Cup')
StaticDecorationFactory = static_category_factory('infinigen/assets/static_assets/source/Decoration')
StaticMirrorFactory = static_category_factory('infinigen/assets/static_assets/source/Mirror')
StaticCeilinglightFactory = static_category_factory('infinigen/assets/static_assets/source/Ceilinglight')
StaticLampFactory = static_category_factory('infinigen/assets/static_assets/source/Lamp')
StaticCounterFactory = static_category_factory('infinigen/assets/static_assets/source/Counter')
StaticPotFactory = static_category_factory('infinigen/assets/static_assets/source/Pot')
StaticTelephoneFactory = static_category_factory('infinigen/assets/static_assets/source/Telephone')
StaticNightstandFactory = static_category_factory('infinigen/assets/static_assets/source/Nightstand')
StaticOvenFactory = static_category_factory('infinigen/assets/static_assets/source/Oven')
StaticTrashcanFactory = static_category_factory('infinigen/assets/static_assets/source/Trashcan')
StaticCoffeemakerFactory = static_category_factory('infinigen/assets/static_assets/source/Coffeemaker')
StaticCurtainFactory = static_category_factory('infinigen/assets/static_assets/source/Curtain')
StaticWallFactory = static_category_factory('infinigen/assets/static_assets/source/Wall')
StaticCartFactory = static_category_factory('infinigen/assets/static_assets/source/Cart')
StaticTableFactory = static_category_factory('infinigen/assets/static_assets/source/Table')
StaticKeyboardFactory = static_category_factory('infinigen/assets/static_assets/source/Keyboard')
StaticSofachairFactory = static_category_factory('infinigen/assets/static_assets/source/Sofachair')
StaticBasketFactory = static_category_factory('infinigen/assets/static_assets/source/Basket')
StaticTeatableFactory = static_category_factory('infinigen/assets/static_assets/source/Teatable')
StaticSideboardcabinetFactory = static_category_factory('infinigen/assets/static_assets/source/Sideboardcabinet')
StaticPanFactory = static_category_factory('infinigen/assets/static_assets/source/Pan')
StaticMonitorFactory = static_category_factory('infinigen/assets/static_assets/source/Monitor')
StaticBoxFactory = static_category_factory('infinigen/assets/static_assets/source/Box')
StaticBackpackFactory = static_category_factory('infinigen/assets/static_assets/source/Backpack')
StaticShelfFactory = static_category_factory('infinigen/assets/static_assets/source/Shelf')
StaticChairFactory = static_category_factory('infinigen/assets/static_assets/source/Chair')
StaticBottleFactory = static_category_factory('infinigen/assets/static_assets/source/Bottle')
StaticSpeakerFactory = static_category_factory('infinigen/assets/static_assets/source/Speaker')
StaticPenFactory = static_category_factory('infinigen/assets/static_assets/source/Pen')
StaticPersonFactory = static_category_factory('infinigen/assets/static_assets/source/Person')
StaticPillowFactory = static_category_factory('infinigen/assets/static_assets/source/Pillow')
StaticMouseFactory = static_category_factory('infinigen/assets/static_assets/source/Mouse')
StaticHearthFactory = static_category_factory('infinigen/assets/static_assets/source/Hearth')
StaticChestofdrawersFactory = static_category_factory('infinigen/assets/static_assets/source/Chestofdrawers')
StaticTowelFactory = static_category_factory('infinigen/assets/static_assets/source/Towel')
StaticPianoFactory = static_category_factory('infinigen/assets/static_assets/source/Piano')
StaticDishwasherFactory = static_category_factory('infinigen/assets/static_assets/source/Dishwasher')
StaticTvFactory = static_category_factory('infinigen/assets/static_assets/source/Tv')
StaticMusicalinstrumentFactory = static_category_factory('infinigen/assets/static_assets/source/Musicalinstrument')
