//#if MOD_THREADED_POLL
static void dm9051_poll_attr(char *headstr, char *tailstr, struct device *dev)
{
	dev_info(dev, "%s - %s%s\n", headstr, THREADED_MODE, tailstr);
}
//#endif
