
Notes
======


```bash

$ cat .idused | sed '/^$/d' | sort | uniq | wc -l

```


```bash
$ while true; do make produce fw=fw/T2p.bin && beep; echo " " && sleep 4; done

$ while true; do make setid && beep; echo " " && sleep 2; done

```

../../../u8g2/tools/font/bdfconv/bdfconv -f 1 -m "32-127" -n Nesobrite_Bk_12pt_r18 -o myfont.c ../../../../Documents/Nesobrite_Bk_12pt_r18.bdf

