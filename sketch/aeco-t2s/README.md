
Notes
======


```bash

$ cat .idused | sed '/^$/d' | sort | uniq | wc -l

```


```bash
$ while true; do make produce fw=fw/T2_V3_7.hex && beep; echo " " && sleep 4; done

$ while true; do make setid && beep; echo " " && sleep 2; done

```

