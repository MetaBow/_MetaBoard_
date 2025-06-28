{
	"patcher" : 	{
		"fileversion" : 1,
		"appversion" : 		{
			"major" : 8,
			"minor" : 6,
			"revision" : 5,
			"architecture" : "x64",
			"modernui" : 1
		}
,
		"classnamespace" : "box",
		"rect" : [ 615.0, 100.0, 654.0, 800.0 ],
		"bglocked" : 0,
		"openinpresentation" : 0,
		"default_fontsize" : 12.0,
		"default_fontface" : 0,
		"default_fontname" : "Arial",
		"gridonopen" : 1,
		"gridsize" : [ 15.0, 15.0 ],
		"gridsnaponopen" : 1,
		"objectsnaponopen" : 1,
		"statusbarvisible" : 2,
		"toolbarvisible" : 1,
		"lefttoolbarpinned" : 0,
		"toptoolbarpinned" : 0,
		"righttoolbarpinned" : 0,
		"bottomtoolbarpinned" : 0,
		"toolbars_unpinned_last_save" : 0,
		"tallnewobj" : 0,
		"boxanimatetime" : 200,
		"enablehscroll" : 1,
		"enablevscroll" : 1,
		"devicewidth" : 0.0,
		"description" : "",
		"digest" : "",
		"tags" : "",
		"style" : "",
		"subpatcher_template" : "",
		"assistshowspatchername" : 0,
		"boxes" : [ 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-1",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "mb.totalextractor.maxpat",
					"numinlets" : 1,
					"numoutlets" : 9,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "", "", "", "", "", "", "", "", "" ],
					"patching_rect" : [ 23.071428571428442, 88.0, 235.0, 535.0 ],
					"viewvisibility" : 1
				}

			}
, 			{
				"box" : 				{
					"fontsize" : 24.0,
					"id" : "obj-38",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 23.071428571428442, 9.500001609325409, 244.0, 36.0 ],
					"style" : "Workshop",
					"text" : "Classification"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-4",
					"maxclass" : "comment",
					"numinlets" : 1,
					"numoutlets" : 0,
					"patching_rect" : [ 23.071428571428442, 47.500001609325409, 164.820514440536499, 21.0 ],
					"style" : "Workshop",
					"text" : "© Davor Vincze"
				}

			}
, 			{
				"box" : 				{
					"id" : "obj-3",
					"maxclass" : "newobj",
					"numinlets" : 2,
					"numoutlets" : 1,
					"outlettype" : [ "" ],
					"patching_rect" : [ 112.0, 663.0, 37.0, 22.0 ],
					"text" : "join 2"
				}

			}
, 			{
				"box" : 				{
					"bgmode" : 0,
					"border" : 0,
					"clickthrough" : 0,
					"enablehscroll" : 0,
					"enablevscroll" : 0,
					"id" : "obj-2",
					"lockeddragscroll" : 0,
					"lockedsize" : 0,
					"maxclass" : "bpatcher",
					"name" : "mb.classifier.maxpat",
					"numinlets" : 1,
					"numoutlets" : 1,
					"offset" : [ 0.0, 0.0 ],
					"outlettype" : [ "" ],
					"patching_rect" : [ 279.0, 114.0, 205.0, 605.0 ],
					"viewvisibility" : 1
				}

			}
 ],
		"lines" : [ 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 1 ],
					"source" : [ "obj-1", 4 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-3", 0 ],
					"source" : [ "obj-1", 0 ]
				}

			}
, 			{
				"patchline" : 				{
					"destination" : [ "obj-2", 0 ],
					"midpoints" : [ 121.5, 687.0, 9.0, 687.0, 9.0, 75.0, 288.5, 75.0 ],
					"source" : [ "obj-3", 0 ]
				}

			}
 ],
		"parameters" : 		{
			"obj-1::obj-1" : [ "live.numbox[5]", "live.numbox[5]", 0 ],
			"obj-1::obj-11" : [ "live.tab[6]", "live.tab[1]", 0 ],
			"obj-1::obj-111" : [ "multislider", "multislider", 0 ],
			"obj-1::obj-15" : [ "live.numbox[6]", "live.numbox[5]", 0 ],
			"obj-1::obj-16" : [ "live.numbox[15]", "live.numbox", 0 ],
			"obj-1::obj-17" : [ "live.text[1]", "live.text[1]", 0 ],
			"obj-2::obj-57" : [ "live.tab[35]", "live.tab[2]", 0 ],
			"obj-2::obj-87" : [ "live.tab[34]", "live.tab", 0 ],
			"obj-2::obj-98" : [ "live.tab[37]", "live.tab", 0 ],
			"parameterbanks" : 			{
				"0" : 				{
					"index" : 0,
					"name" : "",
					"parameters" : [ "-", "-", "-", "-", "-", "-", "-", "-" ]
				}

			}
,
			"parameter_overrides" : 			{
				"obj-2::obj-57" : 				{
					"parameter_longname" : "live.tab[35]"
				}
,
				"obj-2::obj-87" : 				{
					"parameter_longname" : "live.tab[34]"
				}
,
				"obj-2::obj-98" : 				{
					"parameter_longname" : "live.tab[37]"
				}

			}
,
			"inherited_shortname" : 1
		}
,
		"dependency_cache" : [ 			{
				"name" : "fluid.dataset~.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "fluid.labelset~.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "fluid.list2buf.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "fluid.mlpclassifier~.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "fluid.stats.mxo",
				"type" : "iLaX"
			}
, 			{
				"name" : "mb.classifier.maxpat",
				"bootpath" : "~/Documents/Max 8/Packages/MetaBow Toolkit/misc",
				"patcherrelativepath" : "../../Max 8/Packages/MetaBow Toolkit/misc",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "mb.totalextractor.maxpat",
				"bootpath" : "~/Documents/Max 8/Packages/MetaBow Toolkit/misc",
				"patcherrelativepath" : "../../Max 8/Packages/MetaBow Toolkit/misc",
				"type" : "JSON",
				"implicit" : 1
			}
, 			{
				"name" : "parameters.txt",
				"bootpath" : "~/Documents/Max 8/Packages/MetaBow Toolkit/misc",
				"patcherrelativepath" : "../../Max 8/Packages/MetaBow Toolkit/misc",
				"type" : "TEXT",
				"implicit" : 1
			}
, 			{
				"name" : "params.txt",
				"bootpath" : "~/Documents/Max 8/Packages/MetaBow Toolkit/misc",
				"patcherrelativepath" : "../../Max 8/Packages/MetaBow Toolkit/misc",
				"type" : "TEXT",
				"implicit" : 1
			}
 ],
		"autosave" : 0,
		"styles" : [ 			{
				"name" : "Workshop",
				"default" : 				{
					"accentcolor" : [ 0.72156862745098, 0.647058823529412, 0.647058823529412, 1.0 ],
					"bgcolor" : [ 0.725490196078431, 0.776470588235294, 0.517647058823529, 1.0 ],
					"bgfillcolor" : 					{
						"angle" : 270.0,
						"autogradient" : 0.0,
						"color" : [ 1.0, 1.0, 1.0, 1.0 ],
						"color1" : [ 0.752941176470588, 0.76078431372549, 0.023529411764706, 1.0 ],
						"color2" : [ 1.0, 1.0, 1.0, 1.0 ],
						"proportion" : 0.5,
						"type" : "gradient"
					}
,
					"color" : [ 0.152941176470588, 0.427450980392157, 0.172549019607843, 1.0 ],
					"elementcolor" : [ 1.0, 1.0, 1.0, 1.0 ],
					"fontname" : [ "Didot" ],
					"textcolor_inverse" : [ 0.0, 0.0, 0.0, 1.0 ]
				}
,
				"parentstyle" : "",
				"multi" : 0
			}
 ]
	}

}
