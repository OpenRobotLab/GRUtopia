# Copyright (C) 2023, Princeton University.
# This source code is licensed under the BSD 3-Clause license found in the LICENSE file in the root directory of this source tree.

# Authors: Alexander Raistrick

import infinigen.assets.static_assets as static_assets
from infinigen.assets.objects import (
    appliances,
    bathroom,
    decor,
    elements,
    shelves,
    table_decorations,
    tableware,
    wall_decorations,
    windows,
)
from infinigen.core.tags import Semantics


def home_asset_usage():
    """Defines what generators are consider to fulfill what roles in a home setting.

    The primary effect of this to determine what types of objects are returned by the square brackets [ ] operator in home_furniture_constraints

    You can define these however you like - use

    See the `Semantics` class in `infinigen.core.tags` for a list of possible semantics, or add your own.

    """

    # TODO: this whole used_as will be integrated into the constraint language. Currently there are two paralell semantics trees, one to define the tags and one to use them.

    used_as = {}

    # region small objects

    used_as[Semantics.Dishware] = {
        static_assets.StaticPlateFactory,
        static_assets.StaticBowlFactory,
        tableware.WineglassFactory,
        static_assets.StaticPanFactory,
        static_assets.StaticPotFactory,
        static_assets.StaticCupFactory,
    }
    used_as[Semantics.Cookware] = {static_assets.StaticPotFactory, static_assets.StaticPanFactory}
    used_as[Semantics.Utensils] = {
        tableware.SpoonFactory,
        tableware.KnifeFactory,
        tableware.ChopsticksFactory,
        tableware.ForkFactory,
    }

    used_as[Semantics.FoodPantryItem] = {
        tableware.CanFactory,
        tableware.FoodBagFactory,
        tableware.FoodBoxFactory,
        tableware.JarFactory,
        static_assets.StaticBottleFactory,
    }

    used_as[Semantics.TableDisplayItem] = {
        tableware.FruitContainerFactory,
        static_assets.StaticBottleFactory,
        static_assets.StaticBowlFactory,
        static_assets.StaticPotFactory,
    }

    used_as[Semantics.OfficeShelfItem] = {
        elements.NatureShelfTrinketsFactory,
    }

    used_as[Semantics.KitchenCounterItem] = set().union(
        used_as[Semantics.Dishware],
        used_as[Semantics.Cookware],
        {
            tableware.JarFactory,
        },
    )

    used_as[Semantics.BathroomItem] = {
        static_assets.StaticBottleFactory,
        static_assets.StaticBowlFactory,
    }

    used_as[Semantics.ClothDrapeItem] = {
        # objects that can be strewn about / draped over furniture
        # clothes.BlanketFactory,
        static_assets.StaticClothesFactory,
    }

    used_as[Semantics.HandheldItem] = set.union(
        used_as[Semantics.Utensils],
        used_as[Semantics.FoodPantryItem],
        used_as[Semantics.TableDisplayItem],
        used_as[Semantics.OfficeShelfItem],
        used_as[Semantics.ClothDrapeItem],
        used_as[Semantics.Dishware],
    )

    # endregion

    # region furniture

    used_as[Semantics.Sink] = {
        table_decorations.SinkFactory,
        bathroom.BathroomSinkFactory,
        bathroom.StandingSinkFactory,
    }

    used_as[Semantics.Storage] = {
        static_assets.StaticBookshelfFactory,
        static_assets.StaticShelfFactory,
        static_assets.StaticCabinetFactory,
        static_assets.StaticShoecabinetFactory,
    }

    used_as[Semantics.SideTable] = {
        static_assets.StaticSideboardcabinetFactory,
    }

    used_as[Semantics.Table] = set.union(
        used_as[Semantics.SideTable],
        {
            static_assets.StaticTableFactory,
            static_assets.StaticDeskFactory,
            static_assets.StaticTeatableFactory,
        },
    )

    used_as[Semantics.Chair] = {static_assets.StaticChairFactory}

    used_as[Semantics.LoungeSeating] = {
        static_assets.StaticSofachairFactory,
    }

    used_as[Semantics.Seating] = set.union(
        used_as[Semantics.Chair],
        used_as[Semantics.LoungeSeating],
    )

    used_as[Semantics.KitchenAppliance] = {
        static_assets.StaticWashingmachineFactory,
        static_assets.StaticOvenFactory,
        appliances.BeverageFridgeFactory,
        static_assets.StaticMicrowaveFactory,
    }

    used_as[Semantics.KitchenCounter] = {
        shelves.KitchenSpaceFactory,
        shelves.KitchenIslandFactory,
    }

    used_as[Semantics.Bed] = {
        static_assets.StaticBedFactory,
    }

    used_as[Semantics.Furniture] = set().union(
        used_as[Semantics.Storage],
        used_as[Semantics.Table],
        used_as[Semantics.Seating],
        used_as[Semantics.KitchenCounter],
        used_as[Semantics.KitchenAppliance],
        used_as[Semantics.Bed],
        {
            bathroom.StandingSinkFactory,
            static_assets.StaticToiletFactory,
            static_assets.StaticBathtubFactory,
            static_assets.StaticSofachairFactory,
            static_assets.StaticTvstandFactory,
        },
    )

    # endregion furniture

    used_as[Semantics.WallDecoration] = {
        wall_decorations.WallArtFactory,
        static_assets.StaticMirrorFactory,
        wall_decorations.BalloonFactory,
    }

    used_as[Semantics.Door] = {
        elements.doors.GlassPanelDoorFactory,
        elements.doors.LiteDoorFactory,
        elements.doors.LouverDoorFactory,
        elements.doors.PanelDoorFactory,
    }

    used_as[Semantics.Window] = {windows.WindowFactory}

    used_as[Semantics.CeilingLight] = {
        static_assets.StaticCeilinglightFactory,
    }

    used_as[Semantics.Lighting] = set().union(
        used_as[Semantics.CeilingLight],
        {
            static_assets.StaticLampFactory,
        },
    )

    used_as[Semantics.Object] = set().union(
        used_as[Semantics.Furniture],
        used_as[Semantics.Sink],
        used_as[Semantics.Door],
        used_as[Semantics.Window],
        used_as[Semantics.WallDecoration],
        used_as[Semantics.HandheldItem],
        used_as[Semantics.Lighting],
        {
            static_assets.StaticPlantFactory,
            decor.AquariumTankFactory,
            static_assets.StaticTvFactory,
            static_assets.StaticMonitorFactory,
            static_assets.StaticBlanketFactory,
            bathroom.HardwareFactory,
        },
    )

    # region Extra metadata about assets
    # TODO be move outside of the semantics heirarchy and into separate AssetFactory.metadata classvar

    used_as[Semantics.RealPlaceholder] = {}

    used_as[Semantics.AssetAsPlaceholder] = set()

    used_as[Semantics.AssetPlaceholderForChildren] = {
        static_assets.StaticBookshelfFactory,
        static_assets.StaticShelfFactory,
        static_assets.StaticCabinetFactory,
        static_assets.StaticShoecabinetFactory,
        table_decorations.SinkFactory,
        static_assets.StaticTableFactory,
    }

    used_as[Semantics.PlaceholderBBox] = {}

    used_as[Semantics.SingleGenerator] = (
        set()
        .union(
            used_as[Semantics.Dishware],
            used_as[Semantics.Utensils],
            {
                static_assets.StaticCeilinglightFactory,
                static_assets.StaticChairFactory,
            },
        )
        .difference({static_assets.StaticCupFactory})
    )

    used_as[Semantics.NoRotation] = set().union(
        used_as[Semantics.WallDecoration],
        {
            bathroom.HardwareFactory,
            static_assets.StaticCeilinglightFactory,  # rotationally symetric
        },
    )

    used_as[Semantics.NoCollision] = {
        static_assets.StaticBlanketFactory,
    }

    used_as[Semantics.NoChildren] = {
        static_assets.StaticBlanketFactory,
        static_assets.StaticMirrorFactory,
        wall_decorations.WallArtFactory,
        static_assets.StaticCeilinglightFactory,
    }

    # endregion

    return used_as
