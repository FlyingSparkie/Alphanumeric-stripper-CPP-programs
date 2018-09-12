{
    "name" : "",
    "patches" : {
        "@/kapkan-trap-build-1" : {
            "links" : {
                "ByCLEC9H7" : {
                    "id" : "ByCLEC9H7",
                    "input" : {
                        "nodeId" : "BkuWu98rm",
                        "pinKey" : "HyYh1a3LZ"
                    },
                    "output" : {
                        "nodeId" : "B1vYvqLrm",
                        "pinKey" : "H1E8AuSPkZ"
                    }
                },
                "ByskwcISX" : {
                    "id" : "ByskwcISX",
                    "input" : {
                        "nodeId" : "rkM2kqIBQ",
                        "pinKey" : "SJmoRbAUW"
                    },
                    "output" : {
                        "nodeId" : "HysTecUrQ",
                        "pinKey" : "HJU8CE2lW"
                    }
                },
                "SJZOv5IH7" : {
                    "id" : "SJZOv5IH7",
                    "input" : {
                        "nodeId" : "B1iPwcUrX",
                        "pinKey" : "SJmoRbAUW"
                    },
                    "output" : {
                        "nodeId" : "SkKNDcUBX",
                        "pinKey" : "Bk4gU0drwJ-"
                    }
                },
                "SJkZw5UrQ" : {
                    "id" : "SJkZw5UrQ",
                    "input" : {
                        "nodeId" : "HysTecUrQ",
                        "pinKey" : "BJJICN2lW"
                    },
                    "output" : {
                        "nodeId" : "r1isyc8Hm",
                        "pinKey" : "BJ--G1tI-"
                    }
                },
                "SyiYPqUHX" : {
                    "id" : "SyiYPqUHX",
                    "input" : {
                        "nodeId" : "B1vYvqLrm",
                        "pinKey" : "HJG8C_SPkb"
                    },
                    "output" : {
                        "nodeId" : "B1iPwcUrX",
                        "pinKey" : "BJllkG0Ub"
                    }
                },
                "r1ag-9LS7" : {
                    "id" : "r1ag-9LS7",
                    "input" : {
                        "nodeId" : "HysTecUrQ",
                        "pinKey" : "r1jzp_DTZ"
                    },
                    "output" : {
                        "nodeId" : "r1isyc8Hm",
                        "pinKey" : "BJ--G1tI-"
                    }
                },
                "rJsQ3q8H7" : {
                    "id" : "rJsQ3q8H7",
                    "input" : {
                        "nodeId" : "SkKNDcUBX",
                        "pinKey" : "SkSuD6LMb"
                    },
                    "output" : {
                        "nodeId" : "HysTecUrQ",
                        "pinKey" : "HJU8CE2lW"
                    }
                },
                "rJttD58S7" : {
                    "id" : "rJttD58S7",
                    "input" : {
                        "nodeId" : "B1vYvqLrm",
                        "pinKey" : "rJXICuSwyW"
                    },
                    "output" : {
                        "nodeId" : "rkM2kqIBQ",
                        "pinKey" : "BJllkG0Ub"
                    }
                }
            },
            "nodes" : {
                "B1iPwcUrX" : {
                    "boundLiterals" : {
                        "r1xoR-C8-" : "D4",
                        "rJa50WCL-" : "D6"
                    },
                    "id" : "B1iPwcUrX",
                    "position" : {
                        "x" : 408,
                        "y" : 204
                    },
                    "type" : "xod/common-hardware/hc-sr04-ultrasonic-range"
                },
                "B1vYvqLrm" : {
                    "id" : "B1vYvqLrm",
                    "position" : {
                        "x" : 340,
                        "y" : 306
                    },
                    "type" : "xod/core/equal"
                },
                "BkuWu98rm" : {
                    "boundLiterals" : {
                        "B1oqkTnIb" : "D4"
                    },
                    "id" : "BkuWu98rm",
                    "position" : {
                        "x" : 306,
                        "y" : 408
                    },
                    "type" : "xod/common-hardware/led"
                },
                "HysTecUrQ" : {
                    "boundLiterals" : {
                        "B13SCNhl-" : "0.50"
                    },
                    "id" : "HysTecUrQ",
                    "position" : {
                        "x" : 272,
                        "y" : 102
                    },
                    "type" : "xod/core/clock"
                },
                "SkKNDcUBX" : {
                    "boundLiterals" : {
                        "Skre8ROSv1-" : "0.25"
                    },
                    "id" : "SkKNDcUBX",
                    "position" : {
                        "x" : 408,
                        "y" : 102
                    },
                    "type" : "xod/core/delay"
                },
                "r1isyc8Hm" : {
                    "id" : "r1isyc8Hm",
                    "position" : {
                        "x" : 408,
                        "y" : 0
                    },
                    "type" : "xod/common-hardware/button"
                },
                "rkM2kqIBQ" : {
                    "boundLiterals" : {
                        "r1xoR-C8-" : "D4",
                        "rJa50WCL-" : "D6"
                    },
                    "id" : "rkM2kqIBQ",
                    "position" : {
                        "x" : 238,
                        "y" : 204
                    },
                    "type" : "xod/common-hardware/hc-sr04-ultrasonic-range"
                }
            },
            "path" : "@/kapkan-trap-build-1"
        },
        "@/main" : {
            "comments" : {
                "HyZlIT6Bf" : {
                    "content" : "Welcome to XOD again!\n\nDid you complete the built-in tutorial? We recommend you to do it:\n\n»»                                                              ««\n»»»»              [Open Built-in Tutorial Project](xod://actions/open-tutorial)              ««««\n»»                                                              ««\n\nStarting a new project? Delete this comment and start tinkering.\nOr use “File” menu to open an existing project.",
                    "id" : "HyZlIT6Bf",
                    "position" : {
                        "x" : 0,
                        "y" : 0
                    },
                    "size" : {
                        "height" : 255,
                        "width" : 510
                    }
                }
            },
            "path" : "@/main"
        }
    }
} {
    "name" : "",
             "patches":
    {
        "@/kapkan-trap-build-1" : {
            "links" : {
                "ByCLEC9H7" : {
                    "id" : "ByCLEC9H7",
                    "input" : {
                        "nodeId" : "BkuWu98rm",
                        "pinKey" : "HyYh1a3LZ"
                    },
                    "output" : {
                        "nodeId" : "B1vYvqLrm",
                        "pinKey" : "H1E8AuSPkZ"
                    }
                },
                "ByskwcISX" : {
                    "id" : "ByskwcISX",
                    "input" : {
                        "nodeId" : "rkM2kqIBQ",
                        "pinKey" : "SJmoRbAUW"
                    },
                    "output" : {
                        "nodeId" : "HysTecUrQ",
                        "pinKey" : "HJU8CE2lW"
                    }
                },
                "SJZOv5IH7" : {
                    "id" : "SJZOv5IH7",
                    "input" : {
                        "nodeId" : "B1iPwcUrX",
                        "pinKey" : "SJmoRbAUW"
                    },
                    "output" : {
                        "nodeId" : "SkKNDcUBX",
                        "pinKey" : "Bk4gU0drwJ-"
                    }
                },
                "SJkZw5UrQ" : {
                    "id" : "SJkZw5UrQ",
                    "input" : {
                        "nodeId" : "HysTecUrQ",
                        "pinKey" : "BJJICN2lW"
                    },
                    "output" : {
                        "nodeId" : "r1isyc8Hm",
                        "pinKey" : "BJ--G1tI-"
                    }
                },
                "SyiYPqUHX" : {
                    "id" : "SyiYPqUHX",
                    "input" : {
                        "nodeId" : "B1vYvqLrm",
                        "pinKey" : "HJG8C_SPkb"
                    },
                    "output" : {
                        "nodeId" : "B1iPwcUrX",
                        "pinKey" : "BJllkG0Ub"
                    }
                },
                "r1ag-9LS7" : {
                    "id" : "r1ag-9LS7",
                    "input" : {
                        "nodeId" : "HysTecUrQ",
                        "pinKey" : "r1jzp_DTZ"
                    },
                    "output" : {
                        "nodeId" : "r1isyc8Hm",
                        "pinKey" : "BJ--G1tI-"
                    }
                },
                "rJsQ3q8H7" : {
                    "id" : "rJsQ3q8H7",
                    "input" : {
                        "nodeId" : "SkKNDcUBX",
                        "pinKey" : "SkSuD6LMb"
                    },
                    "output" : {
                        "nodeId" : "HysTecUrQ",
                        "pinKey" : "HJU8CE2lW"
                    }
                },
                "rJttD58S7" : {
                    "id" : "rJttD58S7",
                    "input" : {
                        "nodeId" : "B1vYvqLrm",
                        "pinKey" : "rJXICuSwyW"
                    },
                    "output" : {
                        "nodeId" : "rkM2kqIBQ",
                        "pinKey" : "BJllkG0Ub"
                    }
                }
            },
            "nodes" : {
                "B1iPwcUrX" : {
                    "boundLiterals" : {
                        "r1xoR-C8-" : "D4",
                        "rJa50WCL-" : "D6"
                    },
                    "id" : "B1iPwcUrX",
                    "position" : {
                        "x" : 408,
                        "y" : 204
                    },
                    "type" : "xod/common-hardware/hc-sr04-ultrasonic-range"
                },
                "B1vYvqLrm" : {
                    "id" : "B1vYvqLrm",
                    "position" : {
                        "x" : 340,
                        "y" : 306
                    },
                    "type" : "xod/core/equal"
                },
                "BkuWu98rm" : {
                    "boundLiterals" : {
                        "B1oqkTnIb" : "D4"
                    },
                    "id" : "BkuWu98rm",
                    "position" : {
                        "x" : 306,
                        "y" : 408
                    },
                    "type" : "xod/common-hardware/led"
                },
                "HysTecUrQ" : {
                    "boundLiterals" : {
                        "B13SCNhl-" : "0.50"
                    },
                    "id" : "HysTecUrQ",
                    "position" : {
                        "x" : 272,
                        "y" : 102
                    },
                    "type" : "xod/core/clock"
                },
                "SkKNDcUBX" : {
                    "boundLiterals" : {
                        "Skre8ROSv1-" : "0.25"
                    },
                    "id" : "SkKNDcUBX",
                    "position" : {
                        "x" : 408,
                        "y" : 102
                    },
                    "type" : "xod/core/delay"
                },
                "r1isyc8Hm" : {
                    "id" : "r1isyc8Hm",
                    "position" : {
                        "x" : 408,
                        "y" : 0
                    },
                    "type" : "xod/common-hardware/button"
                },
                "rkM2kqIBQ" : {
                    "boundLiterals" : {
                        "r1xoR-C8-" : "D4",
                        "rJa50WCL-" : "D6"
                    },
                    "id" : "rkM2kqIBQ",
                    "position" : {
                        "x" : 238,
                        "y" : 204
                    },
                    "type" : "xod/common-hardware/hc-sr04-ultrasonic-range"
                }
            },
            "path" : "@/kapkan-trap-build-1"
        },
                                  "@/main":
        {
            "comments" : {
                "HyZlIT6Bf" : {
                    "content" : "Welcome to XOD again!\n\nDid you complete the built-in tutorial? We recommend you to do it:\n\n»»                                                              ««\n»»»»              [Open Built-in Tutorial Project](xod://actions/open-tutorial)              ««««\n»»                                                              ««\n\nStarting a new project? Delete this comment and start tinkering.\nOr use “File” menu to open an existing project.",
                    "id" : "HyZlIT6Bf",
                    "position" : {
                        "x" : 0,
                        "y" : 0
                    },
                    "size" : {
                        "height" : 255,
                        "width" : 510
                    }
                }
            },
                         "path" : "@/main"
        }
  